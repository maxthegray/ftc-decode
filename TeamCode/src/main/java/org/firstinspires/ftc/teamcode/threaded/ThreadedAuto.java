package org.firstinspires.ftc.teamcode.threaded;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.threaded.BotState.BallColor;

@Autonomous(name = "Threaded Align and Shoot", group = "Auto")
public class ThreadedAuto extends LinearOpMode {

    private BotState state;
    private DriveThread driveThread;
    private ControlHubI2CThread controlHubI2C;
    private ExpansionHubI2CThread expansionHubI2C;
    private CarouselThread carouselThread;
    private ShooterThread shooterThread;
    private CameraThread cameraThread;
    private ShootSequence shootSequence;

    private ElapsedTime runtime = new ElapsedTime();

    // Alignment
    private static final double BEARING_TOLERANCE = 2.0;
    private static final double MAX_ALIGN_TIME = 5.0;
    private static final double MAX_SHOOT_ORDER_WAIT = 3.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize shared state
        state = new BotState();

        // Initialize threads
        driveThread = new DriveThread(state, hardwareMap);
        controlHubI2C = new ControlHubI2CThread(state, hardwareMap);
        expansionHubI2C = new ExpansionHubI2CThread(state, hardwareMap);
        carouselThread = new CarouselThread(state, hardwareMap);
        shooterThread = new ShooterThread(state, hardwareMap);
        cameraThread = new CameraThread(state, hardwareMap);

        // Initialize shoot sequence
        shootSequence = new ShootSequence(state);

        telemetry.addData("Status", "Initialized - Threaded Auto");
        telemetry.addData("Waiting for", "Camera to start...");
        telemetry.update();

        // Start threads before waitForStart so camera can warm up
        driveThread.start();
        controlHubI2C.start();
        expansionHubI2C.start();
        carouselThread.start();
        shooterThread.start();
        cameraThread.start();

        // Wait for camera to be streaming
        runtime.reset();
        while (!isStarted() && !isStopRequested() && runtime.seconds() < 10) {
            telemetry.addData("Camera", state.getCameraState());
            telemetry.addData("Tag 24", state.isBasketTagVisible() ? "VISIBLE" : "not visible");
            if (state.hasDetectedShootOrder()) {
                telemetry.addData("Shoot Order", "Tag %d detected", state.getShootOrderTagId());
            } else {
                telemetry.addData("Shoot Order", "Waiting...");
            }
            telemetry.update();
            sleep(100);
        }

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            // === PHASE 1: Wait for shoot order tag (21, 22, or 23) ===
            telemetry.addData("Phase", "1/3 - Reading shoot order tag");
            telemetry.update();

            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < MAX_SHOOT_ORDER_WAIT) {
                if (state.hasDetectedShootOrder()) {
                    telemetry.addData("Shoot Order", "Tag %d: %s",
                            state.getShootOrderTagId(),
                            formatShootOrder(state.getDetectedShootOrder()));
                    telemetry.update();
                    sleep(500);  // Give it a moment to confirm
                    break;
                }
                telemetry.addData("Shoot Order", "Searching... (%.1f s)", runtime.seconds());
                telemetry.update();
                sleep(50);
            }

            // === PHASE 2: Align to tag 24 ===
            telemetry.addData("Phase", "2/3 - Aligning to tag 24");
            telemetry.update();

            alignToTag();

            // === PHASE 3: Shoot 3 sorted balls ===
            telemetry.addData("Phase", "3/3 - Shooting");
            telemetry.update();

            shootSequence.start();

            // Wait for shoot sequence to complete
            while (opModeIsActive() && shootSequence.isActive()) {
                shootSequence.update();
                updateTelemetry();
                sleep(20);
            }

            telemetry.addData("Status", "Complete!");
            telemetry.update();
            sleep(2000);
        }

        // Clean up - end all threads
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

    private void alignToTag() {
        runtime.reset();

        // Enable auto-align
        state.setAutoAlignEnabled(true);

        while (opModeIsActive() && runtime.seconds() < MAX_ALIGN_TIME) {
            if (state.isBasketTagVisible()) {
                double bearing = state.getTagBearing();

                telemetry.addData("Tag 24", "VISIBLE");
                telemetry.addData("Bearing", "%.1f°", bearing);
                telemetry.addData("Range", "%.1f in", state.getTagRange());

                if (Math.abs(bearing) < BEARING_TOLERANCE) {
                    telemetry.addData("Status", "ALIGNED!");
                    telemetry.update();
                    sleep(300);  // Hold alignment briefly
                    break;
                }
            } else {
                telemetry.addData("Tag 24", "NOT VISIBLE");
                telemetry.addData("Status", "Searching...");
            }

            telemetry.addData("Time", "%.1f / %.1f s", runtime.seconds(), MAX_ALIGN_TIME);
            telemetry.update();
            sleep(50);
        }

        // Disable auto-align and stop
        state.setAutoAlignEnabled(false);
        state.setDriveInput(0, 0, 0);

        telemetry.addData("Align", "Complete (%.1f sec)", runtime.seconds());
        telemetry.update();
    }

    private void updateTelemetry() {
        telemetry.addData("Runtime", "%.1f s", runtime.seconds());

        // Shoot sequence status
        if (shootSequence.isActive()) {
            telemetry.addLine("=== SHOOT SEQUENCE ===");
            telemetry.addData("Ball", "%d/%d", shootSequence.getBallIndex() + 1, shootSequence.getTotalBalls());
            telemetry.addData("State", shootSequence.getStateName());
            BallColor target = shootSequence.getCurrentTarget();
            if (target != null) {
                telemetry.addData("Target", target);
            }
            BallColor[] order = shootSequence.getShootOrder();
            if (order != null) {
                telemetry.addData("Order", formatShootOrder(order));
            }
        }

        // Tag info
        telemetry.addLine("=== APRILTAG ===");
        telemetry.addData("Tag 24", state.isBasketTagVisible() ? "VISIBLE" : "not visible");
        if (state.isBasketTagVisible()) {
            telemetry.addData("Range", "%.1f in", state.getTagRange());
            telemetry.addData("Bearing", "%.1f°", state.getTagBearing());
        }

        // Shooter
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Target", "%.0f deg/s", state.getShooterTargetVelocity());
        telemetry.addData("Current", "%.0f deg/s", state.getShooterCurrentVelocity());
        telemetry.addData("Ready", state.isShooterReady() ? "YES" : "NO");

        // Carousel
        telemetry.addLine("=== CAROUSEL ===");
        telemetry.addData("Settled", state.isCarouselSettled());
        BallColor[] positions = state.getAllPositions();
        telemetry.addData("Positions", "I:%s BL:%s BR:%s",
                positions[BotState.POS_INTAKE],
                positions[BotState.POS_BACK_LEFT],
                positions[BotState.POS_BACK_RIGHT]);

        telemetry.update();
    }

    private String formatShootOrder(BallColor[] order) {
        if (order == null || order.length < 3) return "UNKNOWN";
        return String.format("%s-%s-%s",
                order[0].toString().charAt(0),
                order[1].toString().charAt(0),
                order[2].toString().charAt(0));
    }
}