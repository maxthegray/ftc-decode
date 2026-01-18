package org.firstinspires.ftc.teamcode.threaded;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto - BLUE Simple", group = "Auto")
public class AutoBlue extends LinearOpMode {

    // Blue alliance uses tag 20
    private static final int BASKET_TAG_ID = CameraThread.TAG_BLUE_BASKET;

    // Alignment tuning
    private static final double ALIGN_P = 0.015;
    private static final double ALIGN_DEADBAND = 2.0;      // degrees - "good enough"
    private static final double ALIGN_MIN_POWER = 0.10;
    private static final double ALIGN_MAX_POWER = 0.35;
    private static final long ALIGN_TIMEOUT_MS = 3000;     // give up after 3 sec
    private static final long ALIGN_STABLE_MS = 300;       // must be aligned for 300ms

    private BotState state;
    private DriveThread driveThread;
    private ControlHubI2CThread controlHubI2C;
    private ExpansionHubI2CThread expansionHubI2C;
    private CarouselThread carouselThread;
    private ShooterThread shooterThread;
    private CameraThread cameraThread;
    private ShootSequence shootSequence;

    private ElapsedTime runtime;

    @Override
    public void runOpMode() {
        runtime = new ElapsedTime();

        // Initialize state
        state = new BotState();

        // Disable auto-index during autonomous (we control everything)
        state.setAutoIndexEnabled(false);

        // Initialize threads
        driveThread = new DriveThread(state, hardwareMap);
        controlHubI2C = new ControlHubI2CThread(state, hardwareMap);
        expansionHubI2C = new ExpansionHubI2CThread(state, hardwareMap);
        carouselThread = new CarouselThread(state, hardwareMap);
        shooterThread = new ShooterThread(state, hardwareMap);
        cameraThread = new CameraThread(state, hardwareMap, BASKET_TAG_ID);

        // Initialize shoot sequence
        shootSequence = new ShootSequence(state);

        telemetry.addData("Alliance", "BLUE");
        telemetry.addData("Status", "Initialized - Waiting for Start");
        telemetry.addData("Basket Tag", BASKET_TAG_ID);
        telemetry.update();

        // Wait for start
        waitForStart();
        runtime.reset();

        if (!opModeIsActive()) return;

        // Start threads
        driveThread.start();
        controlHubI2C.start();
        expansionHubI2C.start();
        carouselThread.start();
        shooterThread.start();
        cameraThread.start();

        // Give sensors time to initialize
        safeSleep(500);

        // === STEP 1: Wait for camera to see tag ===
        telemetry.addData("Step", "1 - Waiting for AprilTag...");
        telemetry.update();

        if (!waitForTag(5000)) {
            telemetry.addData("ERROR", "Tag not found! Aborting.");
            telemetry.update();
            cleanup();
            return;
        }

        // === STEP 2: Align to basket ===
        telemetry.addData("Step", "2 - Aligning to basket...");
        telemetry.update();

        boolean aligned = alignToBasket();
        if (!aligned) {
            telemetry.addData("WARNING", "Alignment timed out, continuing anyway");
            telemetry.update();
        }

        // Stop rotation
        state.setDriveInput(0, 0, 0);
        safeSleep(200);

        // === STEP 3: Shoot sequence ===
        telemetry.addData("Step", "3 - Shooting...");
        telemetry.update();

        shootSequence.start();

        while (opModeIsActive() && shootSequence.isActive()) {
            shootSequence.update();

            // Continuously re-align while shooting
            if (state.isBasketTagVisible()) {
                double bearing = state.getTagBearing();
                if (Math.abs(bearing) > ALIGN_DEADBAND) {
                    double rotate = calculateAlignPower(bearing);
                    state.setDriveInput(0, 0, rotate);
                } else {
                    state.setDriveInput(0, 0, 0);
                }
            }

            // Telemetry
            telemetry.addData("Step", "3 - Shooting");
            telemetry.addData("Ball", "%d/%d",
                    shootSequence.getBallIndex() + 1,
                    shootSequence.getTotalBalls());
            telemetry.addData("State", shootSequence.getStateName());
            telemetry.addData("Target", shootSequence.getCurrentTarget());
            telemetry.addData("Shooter Ready", state.isShooterReady());
            telemetry.addData("Tag Visible", state.isBasketTagVisible());
            if (state.isBasketTagVisible()) {
                telemetry.addData("Bearing", "%.1f°", state.getTagBearing());
                telemetry.addData("Range", "%.1f in", state.getTagRange());
            }
            telemetry.update();

            safeSleep(20);
        }

        // === DONE ===
        state.setShooterTargetVelocity(0);
        state.setDriveInput(0, 0, 0);

        telemetry.addData("Step", "DONE!");
        telemetry.addData("Runtime", "%.1f sec", runtime.seconds());
        telemetry.update();

        // Keep displaying until auto ends
        while (opModeIsActive()) {
            safeSleep(100);
        }

        cleanup();
    }

    /**
     * Wait for the basket tag to be visible.
     * @param timeoutMs Maximum time to wait
     * @return true if tag found, false if timeout
     */
    private boolean waitForTag(long timeoutMs) {
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.milliseconds() < timeoutMs) {
            if (state.isBasketTagVisible()) {
                return true;
            }

            telemetry.addData("Step", "1 - Waiting for AprilTag");
            telemetry.addData("Camera State", state.getCameraState());
            telemetry.addData("Detections", state.getDetectionCount());
            telemetry.addData("Time", "%.1f / %.1f sec",
                    timer.milliseconds() / 1000.0,
                    timeoutMs / 1000.0);
            telemetry.update();

            safeSleep(50);
        }

        return false;
    }

    /**
     * Align robot to face the basket tag.
     * @return true if aligned, false if timeout
     */
    private boolean alignToBasket() {
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime stableTimer = new ElapsedTime();
        boolean wasAligned = false;

        while (opModeIsActive() && timer.milliseconds() < ALIGN_TIMEOUT_MS) {
            if (!state.isBasketTagVisible()) {
                // Lost tag, keep last rotation command briefly
                stableTimer.reset();
                wasAligned = false;
                safeSleep(20);
                continue;
            }

            double bearing = state.getTagBearing();
            double absBearing = Math.abs(bearing);

            if (absBearing <= ALIGN_DEADBAND) {
                // We're aligned
                state.setDriveInput(0, 0, 0);

                if (!wasAligned) {
                    stableTimer.reset();
                    wasAligned = true;
                }

                // Check if we've been stable long enough
                if (stableTimer.milliseconds() >= ALIGN_STABLE_MS) {
                    return true;
                }
            } else {
                // Not aligned, rotate
                wasAligned = false;
                double rotate = calculateAlignPower(bearing);
                state.setDriveInput(0, 0, rotate);
            }

            // Telemetry
            telemetry.addData("Step", "2 - Aligning");
            telemetry.addData("Bearing", "%.1f° (target: ±%.1f°)", bearing, ALIGN_DEADBAND);
            telemetry.addData("Aligned", wasAligned ? "YES (stable: %.0fms)" : "NO",
                    stableTimer.milliseconds());
            telemetry.addData("Time", "%.1f / %.1f sec",
                    timer.milliseconds() / 1000.0,
                    ALIGN_TIMEOUT_MS / 1000.0);
            telemetry.update();

            safeSleep(20);
        }

        return false;
    }

    /**
     * Calculate rotation power for alignment.
     */
    private double calculateAlignPower(double bearing) {
        double power = ALIGN_P * bearing;

        // Apply minimum power
        if (Math.abs(power) < ALIGN_MIN_POWER) {
            power = Math.signum(bearing) * ALIGN_MIN_POWER;
        }

        // Clamp to max
        power = Math.max(-ALIGN_MAX_POWER, Math.min(ALIGN_MAX_POWER, power));

        return power;
    }

    /**
     * Sleep that checks opModeIsActive.
     */
    private void safeSleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Clean up all threads.
     */
    private void cleanup() {
        state.endThreads();

        try {
            driveThread.join(500);
            controlHubI2C.join(500);
            expansionHubI2C.join(500);
            carouselThread.join(500);
            shooterThread.join(500);
            cameraThread.join(500);
        } catch (InterruptedException e) {
            // Ignore
        }
    }
}