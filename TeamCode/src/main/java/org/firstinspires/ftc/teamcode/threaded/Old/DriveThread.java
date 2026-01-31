package org.firstinspires.ftc.teamcode.threaded.Old;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DriveThread extends Thread {

    private final BotState state;
    private final Follower follower;

    // Inline PID variables for auto-align
    private double integralSum = 0;
    private double lastError = 0;
    private boolean hasLastError = false;
    private final ElapsedTime pidTimer = new ElapsedTime();

    // PID output limits
    private static final double OUTPUT_MIN = -1.0;
    private static final double OUTPUT_MAX = 1.0;
    private static final double INTEGRAL_LIMIT = 0.3;

    // Track if we were aligning last loop (to reset PID when starting fresh)
    private boolean wasAligning = false;

    public DriveThread(BotState state, HardwareMap hardwareMap) {
        this.state = state;
        this.setPriority(Thread.MAX_PRIORITY);

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleOpDrive();
    }

    @Override
    public void run() {
        while (!state.shouldKillThreads()) {

            // Check for pose update request from AprilTag
            if (state.isPoseUpdateRequested()) {
                Pose tagPose = state.getTagCalculatedPose();
                if (tagPose != null) {
                    follower.setPose(tagPose);
                }
                state.clearPoseUpdateRequest();
            }


            // Get drive input from state
            double forward = state.getDriveForward();
            double strafe = state.getDriveStrafe();
            double rotate = state.getDriveRotate();

            // Auto-align using PID
            boolean isAligning = state.isAutoAlignEnabled() && state.isBasketTagVisible();

            if (isAligning) {
                // Reset PID if we just started aligning
                if (!wasAligning) {
                    resetPID();
                }

                // Use corrected bearing to actual target point (accounts for offset behind tag)
                double bearing = state.getTargetBearing();

                // PID outputs rotation power to minimize bearing
                rotate = calculatePID(bearing);

            } else if (wasAligning) {
                // Just stopped aligning - reset PID for next time
                resetPID();
            }

            wasAligning = isAligning;

            // Apply to follower
            follower.setTeleOpDrive(forward, strafe, rotate, true);
            follower.update();

            // Update state with current pose
            state.setCurrentPose(follower.getPose());
            state.setFollowingPath(follower.isBusy());

            try {
                Thread.sleep(BotState.DRIVE_UPDATE_MS);
            } catch (InterruptedException e) {
                break;
            }
        }
    }

    private double calculatePID(double error) {
        double dt = pidTimer.seconds();
        pidTimer.reset();

        double kP = BotState.ALIGN_P;
        double kI = BotState.ALIGN_I;
        double kD = BotState.ALIGN_D;
        double deadband = BotState.ALIGN_DEADBAND;

        if (Math.abs(error) < deadband) {
            integralSum = 0;
            lastError = error;
            return 0.0;
        }


        // Proportional
        double p = kP * error;

        // Integral (with anti-windup)
        if (dt > 0 && dt < 1.0) {  // Sanity check dt
            integralSum += error * dt;
            integralSum = clamp(integralSum, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        }
        double i = kI * integralSum;

        // Derivative
        double d = 0;
        if (hasLastError && dt > 0 && dt < 1.0) {
            d = kD * (error - lastError) / dt;
        }

        lastError = error;
        hasLastError = true;

        // Sum and clamp output
        double output = p + i + d;
        return clamp(output, OUTPUT_MIN, OUTPUT_MAX);
    }


    private void resetPID() {
        integralSum = 0;
        lastError = 0;
        hasLastError = false;
        pidTimer.reset();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public Follower getFollower() {
        return follower;
    }

    // Getters for telemetry/debugging
    public double getLastError() {
        return lastError;
    }

    public double getIntegralSum() {
        return integralSum;
    }
}