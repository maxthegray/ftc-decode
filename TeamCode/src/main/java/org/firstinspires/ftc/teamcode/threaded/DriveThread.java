package org.firstinspires.ftc.teamcode.threaded;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Handles drivetrain and auto-align.
 *
 * Loop order matches the tuner that produced smooth alignment:
 *   1. follower.update()          — integrate odometry first
 *   2. read fresh pose & heading  — so interpolation uses current data
 *   3. compute PID                — bearing is accurate, no one-cycle lag
 *   4. setTeleOpDrive()           — apply powers last
 *
 * ── KEY TIMING NOTE ──────────────────────────────────────────────────────
 * The tuner OpMode runs at ~50-100Hz naturally (telemetry.update() blocks).
 * This thread must run at a similar rate — NOT as fast as possible.
 *
 * Running too fast (1000Hz+) causes:
 *   - Derivative spikes when camera frames arrive (small error jump / tiny dt)
 *   - follower.update() called with near-zero encoder deltas
 *   - Jittery output even with good PID gains
 *
 * TARGET_LOOP_MS = 10 → ~100Hz. Midpoint between 5ms (too fast/oscillating)
 * and 15ms (too slow). Measure the tuner's actual loop time and replace.
 */
public class DriveThread extends Thread {

    private final SensorState state;
    private final Follower follower;

    // ── Loop rate limiter ────────────────────────────────────────────────
    // Targets a consistent loop rate matching the tuner's effective rate.
    // Uses busy-wait (not Thread.sleep) to avoid Android scheduler stretching.
    // TODO: measure tuner's actual loop time and set this to match exactly.
    // 5ms was too fast (oscillation), 15ms was too slow. Trying 10ms.
    private static final double TARGET_LOOP_MS = 10.0;
    private final ElapsedTime loopTimer = new ElapsedTime();

    // ── PID for auto-align ───────────────────────────────────────────────
    private double integralSum = 0;
    private double lastError = 0;
    private boolean hasLastError = false;

    private static final double OUTPUT_MIN = -1.0;
    private static final double OUTPUT_MAX = 1.0;
    private static final double INTEGRAL_LIMIT = 0.3;

    // Derivative low-pass filter — read from SensorState so it's tunable live.
    private double filteredDerivative = 0;

    // ── Tag-visibility debounce ──────────────────────────────────────────
    // Don't reset PID when the camera drops a single frame.
    private boolean wasAligning = false;
    private int tagLostFrames = 0;
    private static final int TAG_LOST_THRESHOLD = 6;  // ~60ms at 100Hz

    // ── Tuner support ────────────────────────────────────────────────────
    // Volatile diagnostics readable from the OpMode thread for telemetry.
    private volatile double diagPidOutput = 0;
    private volatile double diagRawBearing = 0;
    private volatile double diagInterpBearing = 0;
    private volatile double diagHeadingDelta = 0;
    private volatile double diagLoopMs = 0;
    private volatile double diagDt = 0;

    // When true, PID uses raw bearing instead of interpolated (for A/B comparison)
    private volatile boolean useRawBearing = false;

    // External PID reset request (set by tuner when toggling modes)
    private volatile boolean pidResetRequested = false;

    public double getDiagPidOutput()      { return diagPidOutput; }
    public double getDiagRawBearing()     { return diagRawBearing; }
    public double getDiagInterpBearing()  { return diagInterpBearing; }
    public double getDiagHeadingDelta()   { return diagHeadingDelta; }
    public double getDiagLoopMs()         { return diagLoopMs; }
    public double getDiagDt()             { return diagDt; }

    public void setUseRawBearing(boolean raw) { this.useRawBearing = raw; }
    public boolean getUseRawBearing()         { return useRawBearing; }
    public void requestPidReset()             { this.pidResetRequested = true; }

    public DriveThread(SensorState state, HardwareMap hardwareMap) {
        this.state = state;
        this.setPriority(Thread.MAX_PRIORITY);

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleOpDrive();
    }

    @Override
    public void run() {
        loopTimer.reset();

        while (!state.shouldKillThreads()) {

            // ── Loop rate limiter (busy-wait) ────────────────────────────
            while (loopTimer.milliseconds() < TARGET_LOOP_MS) {
                Thread.yield();
            }
            diagLoopMs = loopTimer.milliseconds();
            loopTimer.reset();

            // ── External PID reset (from tuner) ──────────────────────────
            if (pidResetRequested) {
                resetPID();
                pidResetRequested = false;
            }

            // ── 1. Update odometry FIRST so pose is fresh ────────────────
            follower.update();

            // ── 2. Read fresh pose and push it to shared state ───────────
            Pose currentPose = follower.getPose();
            state.setCurrentPose(currentPose);
            double currentHeading = currentPose.getHeading();

            // Handle pose reset from AprilTag
            if (state.isPoseUpdateRequested()) {
                Pose tagPose = state.getTagCalculatedPose();
                if (tagPose != null) {
                    follower.setPose(tagPose);
                }
                state.clearPoseUpdateRequest();
            }

            // ── 3. Build drive command ───────────────────────────────────
            double forward = state.getDriveForward();
            double strafe = state.getDriveStrafe();
            double rotate = state.getDriveRotate();

            // Update bearing diagnostics every loop (even when not aligning)
            if (state.isBasketTagVisible()) {
                diagRawBearing = state.getRawTargetBearing();
                diagInterpBearing = state.getInterpolatedBearing(currentHeading);
                diagHeadingDelta = Math.toDegrees(currentHeading - state.getHeadingAtLastFrame());
            }

            // Auto-align — debounced tag visibility
            boolean tagVisible = state.isAutoAlignEnabled() && state.isBasketTagVisible();

            if (tagVisible) {
                tagLostFrames = 0;
                if (!wasAligning) resetPID();

                double error;
                if (useRawBearing) {
                    error = state.getTargetBearing();
                } else {
                    error = state.getInterpolatedBearing(currentHeading);
                }

                rotate = calculatePID(error);
                diagPidOutput = rotate;
                wasAligning = true;
            } else if (wasAligning) {
                tagLostFrames++;
                if (tagLostFrames > TAG_LOST_THRESHOLD) {
                    resetPID();
                    wasAligning = false;
                    diagPidOutput = 0;
                }
            } else {
                diagPidOutput = 0;
            }

            // ── 4. Apply drive powers LAST ───────────────────────────────
            follower.setTeleOpDrive(forward, strafe, rotate, false);

            state.setFollowingPath(follower.isBusy());
        }
    }

    private double calculatePID(double error) {
        // Use fixed dt to avoid derivative spikes from thread scheduling jitter.
        // DriveThread targets TARGET_LOOP_MS, so use that as a constant.
        double dt = TARGET_LOOP_MS / 1000.0;
        diagDt = dt;

        if (Math.abs(error) < SensorState.ALIGN_DEADBAND) {
            integralSum = 0;
            lastError = error;
            filteredDerivative = 0;
            return 0.0;
        }

        double p = SensorState.ALIGN_P * error;

        integralSum += error * dt;
        integralSum = clamp(integralSum, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        double i = SensorState.ALIGN_I * integralSum;

        double d = 0;
        if (hasLastError) {
            double rawDerivative = (error - lastError) / dt;
            // Low-pass filter: smooths derivative spikes from camera frame arrivals
            filteredDerivative = SensorState.ALIGN_D_ALPHA * filteredDerivative
                    + (1.0 - SensorState.ALIGN_D_ALPHA) * rawDerivative;
            d = SensorState.ALIGN_D * filteredDerivative;
        }

        lastError = error;
        hasLastError = true;

        return clamp(p + i + d, OUTPUT_MIN, OUTPUT_MAX);
    }

    private void resetPID() {
        integralSum = 0;
        lastError = 0;
        hasLastError = false;
        filteredDerivative = 0;
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