package org.firstinspires.ftc.teamcode.threaded.Old;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Handles drivetrain and auto-align.
 */
public class DriveThread extends Thread {

    private final      SensorState state;
    private final Follower follower;

    // PID for auto-align
    private double integralSum = 0;
    private double lastError = 0;
    private boolean hasLastError = false;
    private final ElapsedTime pidTimer = new ElapsedTime();

    private static final double OUTPUT_MIN = -1.0;
    private static final double OUTPUT_MAX = 1.0;
    private static final double INTEGRAL_LIMIT = 0.3;

    private boolean wasAligning = false;

    public DriveThread(     SensorState state, HardwareMap hardwareMap) {
        this.state = state;
        this.setPriority(Thread.MAX_PRIORITY);

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleOpDrive();
    }

    @Override
    public void run() {
        while (!state.shouldKillThreads()) {
            // Handle pose update from AprilTag
            if (state.isPoseUpdateRequested()) {
                Pose tagPose = state.getTagCalculatedPose();
                if (tagPose != null) {
                    follower.setPose(tagPose);
                }
                state.clearPoseUpdateRequest();
            }

            // Get drive input
            double forward = state.getDriveForward();
            double strafe = state.getDriveStrafe();
            double rotate = state.getDriveRotate();

            // Auto-align
            boolean isAligning = state.isAutoAlignEnabled() && state.isBasketTagVisible();

            if (isAligning) {
                if (!wasAligning) resetPID();
                rotate = calculatePID(state.getTargetBearing());
            } else if (wasAligning) {
                resetPID();
            }
            wasAligning = isAligning;

            // Apply
            follower.setTeleOpDrive(forward, strafe, rotate, false);
            follower.update();

            // Update state
            state.setCurrentPose(follower.getPose());
            state.setFollowingPath(follower.isBusy());

            try {
                Thread.sleep(     SensorState.DRIVE_UPDATE_MS);
            } catch (InterruptedException e) {
                break;
            }
        }
    }

    private double calculatePID(double error) {
        double dt = pidTimer.seconds();
        pidTimer.reset();

        if (Math.abs(error) <      SensorState.ALIGN_DEADBAND) {
            integralSum = 0;
            lastError = error;
            return 0.0;
        }

        double p =      SensorState.ALIGN_P * error;

        if (dt > 0 && dt < 1.0) {
            integralSum += error * dt;
            integralSum = clamp(integralSum, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        }
        double i =      SensorState.ALIGN_I * integralSum;

        double d = 0;
        if (hasLastError && dt > 0 && dt < 1.0) {
            d =      SensorState.ALIGN_D * (error - lastError) / dt;
        }

        lastError = error;
        hasLastError = true;

        return clamp(p + i + d, OUTPUT_MIN, OUTPUT_MAX);
    }

    private void resetPID() {
        integralSum = 0;
        lastError = 0;
        hasLastError = false;
        pidTimer.reset();
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    public void cancelPath() {
        follower.breakFollowing();
    }

    public Follower getFollower() {
        return follower;
    }
}