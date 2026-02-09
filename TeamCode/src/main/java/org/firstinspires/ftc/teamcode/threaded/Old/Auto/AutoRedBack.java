//package org.firstinspires.ftc.teamcode.threaded.Old.Auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.threaded.Old.SensorState;
//import org.firstinspires.ftc.teamcode.threaded.Old.CameraThread;
//import org.firstinspires.ftc.teamcode.threaded.Old.DriveThread;
//import org.firstinspires.ftc.teamcode.threaded.Old.ShooterThread;
//
//@Autonomous(name = "Back Red Auto)", group = "Auto")
//public class AutoRedBack extends LinearOpMode {
//
//    private static final int BASKET_TAG_ID = CameraThread.TAG_RED_BASKET;
//
//    // Drive parameters
//    private static final double DRIVE_POWER = 0.5;
//    private static final double STRAFE_POWER = 0.5;
//    private static final double BACKUP_TIME_SEC = 1.0;      // time to drive backward
//    private static final double STRAFE_TIME_SEC = 0.75;      // time to strafe right
//    private static final double AUTO_DRIVE_TIMEOUT_SEC = 15.0; // start strafing after 15 seconds
//
//    // Shooter/carousel parameters
//    private static final double DEFAULT_VELOCITY = 142;       // if tag not visible
//    private static final long TAG_TIMEOUT_MS = 3000;        // wait for tag
//    private static final long SPIN_UP_MS = 1500;            // time to spin up shooter
//    private static final long POST_KICK_MS = 500;           // wait after kick
//    private static final long CAROUSEL_SETTLE_MS = 800;     // wait for carousel to rotate
//
//    private SensorState state;
//    private DriveThread driveThread;
//    private CarouselThread carouselThread;
//    private ShooterThread shooterThread;
//    private CameraThread cameraThread;
//
//    private ElapsedTime masterTimer; // Tracks time since start
//
//    @Override
//    public void runOpMode() {
//        // Initialize
//        state = new SensorState();
//        state.setAutoIndexEnabled(false);
//
//        driveThread = new DriveThread(state, hardwareMap);
//        carouselThread = new CarouselThread(state, hardwareMap);
//        shooterThread = new ShooterThread(state, hardwareMap);
//        cameraThread = new CameraThread(state, hardwareMap, BASKET_TAG_ID);
//
//        telemetry.addData("Alliance", "RED (Tag %d)", BASKET_TAG_ID);
//        telemetry.addData("Status", "Ready");
//        telemetry.update();
//
//        waitForStart();
//
//        // Start master timer immediately
//        masterTimer = new ElapsedTime();
//
//        // Start threads
//        driveThread.start();
//        carouselThread.start();
//        shooterThread.start();
//        cameraThread.start();
//
//        // === STEP 1: Drive BACKWARD ===
//        ElapsedTime timer = new ElapsedTime();
//        timer.reset();
//
//        while (opModeIsActive() && timer.seconds() < BACKUP_TIME_SEC) {
//            state.setDriveInput(-DRIVE_POWER, 0, 0); // Negative for backward
//
//            telemetry.addData("Step", "Driving backward");
//            telemetry.addData("Time", "%.1f / %.1f sec", timer.seconds(), BACKUP_TIME_SEC);
//            telemetry.update();
//        }
//        state.setDriveInput(0, 0, 0);
//        safeSleep(200);
//
//        // === STEP 2: Wait for tag to get distance ===
//        telemetry.addData("Step", "Looking for tag...");
//        telemetry.update();
//
//        timer.reset();
//        while (opModeIsActive() && !state.isBasketTagVisible()
//                && timer.milliseconds() < TAG_TIMEOUT_MS
//                && !shouldStartStrafing()) {
//            telemetry.addData("Step", "Looking for tag...");
//            telemetry.addData("Time", "%.1f / %.1f sec", timer.seconds(), TAG_TIMEOUT_MS / 1000.0);
//            telemetry.addData("Auto Timer", "%.1f / %.1f sec", masterTimer.seconds(), AUTO_DRIVE_TIMEOUT_SEC);
//            telemetry.update();
//            safeSleep(50);
//        }
//
//        // Check if we should skip to strafing
//        if (!shouldStartStrafing()) {
//            // Set shooter velocity based on distance
//            if (state.isBasketTagVisible()) {
//                double range = state.getTagRange();
//                state.setAdjustedVelocity(range);
//                sleep(500);
//                state.setShooterTargetVelocity(state.getShooterTargetVelocity() - 2);
//                telemetry.addData("Range", "%.1f in", range);
//                telemetry.addData("Velocity", "%.0f deg/s", state.getShooterTargetVelocity());
//                safeSleep(1000);
//
//            } else {
//                state.setShooterTargetVelocity(DEFAULT_VELOCITY);
//
//                telemetry.addData("Range", "TAG NOT FOUND - using default");
//                telemetry.addData("Velocity", "%.0f deg/s", DEFAULT_VELOCITY);
//            }
//            telemetry.update();
//
//            // === STEP 3: Spin up shooter ===
//            telemetry.addData("Step", "Spinning up...");
//            telemetry.update();
//            safeSleep(SPIN_UP_MS);
//
//            // === STEP 4: Shoot 3 balls ===
//            for (int i = 1; i <= 3 && !shouldStartStrafing(); i++) {
//
//                telemetry.addData("Step", "Shooting ball %d/3", i);
//                telemetry.addData("Velocity", "%.0f deg/s", state.getShooterTargetVelocity());
//                telemetry.addData("Auto Timer", "%.1f / %.1f sec", masterTimer.seconds(), AUTO_DRIVE_TIMEOUT_SEC);
//                telemetry.update();
//
//                // Wait for shooter ready
//                while (opModeIsActive() && !state.isShooterReady() && !shouldStartStrafing()) {
//                    safeSleep(20);
//                }
//
//                if (shouldStartStrafing()) break;
//
//                // Kick
//                state.requestKick();
//
//                // Wait for kick to complete
//                safeSleep(POST_KICK_MS);
//                while (opModeIsActive() && state.isKickerUp() && !shouldStartStrafing()) {
//                    safeSleep(20);
//                }
//
//                // Rotate carousel for next ball (except after last shot)
//                if (i < 3 && !shouldStartStrafing()) {
//                    state.setCarouselCommand(SensorState.CarouselCommand.ROTATE_RIGHT);
//                    safeSleep(CAROUSEL_SETTLE_MS);
//                    while (opModeIsActive() && !state.isCarouselSettled() && !shouldStartStrafing()) {
//                        safeSleep(20);
//                    }
//                }
//            }
//        }
//
//        // === STEP 5: STRAFE RIGHT ===
//        telemetry.addData("Step", "Starting strafe at %.1f sec", masterTimer.seconds());
//        telemetry.update();
//
//        timer.reset();
//        while (opModeIsActive() && timer.seconds() < STRAFE_TIME_SEC) {
//            state.setDriveInput(0, STRAFE_POWER, 0); // Positive strafe = right
//
//            telemetry.addData("Step", "Strafing right");
//            telemetry.addData("Time", "%.1f / %.1f sec", timer.seconds(), STRAFE_TIME_SEC);
//            telemetry.update();
//        }
//        state.setDriveInput(0, 0, 0);
//        safeSleep(200);
//
//        // Stop shooter
//        state.setShooterTargetVelocity(0);
//
//        telemetry.addData("Step", "Done!");
//        telemetry.update();
//
//        // Cleanup
//        state.endThreads();
//        try {
//            driveThread.join(500);
//            carouselThread.join(500);
//            shooterThread.join(500);
//            cameraThread.join(500);
//        } catch (InterruptedException e) {
//            // Ignore
//        }
//    }
//
//    /**
//     * Returns true if 15 seconds have elapsed and we should start strafing
//     */
//    private boolean shouldStartStrafing() {
//        return masterTimer.seconds() >= AUTO_DRIVE_TIMEOUT_SEC;
//    }
//
//    private void safeSleep(long ms) {
//        try {
//            Thread.sleep(ms);
//        } catch (InterruptedException e) {
//            Thread.currentThread().interrupt();
//        }
//    }
//}