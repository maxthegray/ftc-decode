package org.firstinspires.ftc.teamcode.threaded;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.threaded.BotState.BallColor;
import org.firstinspires.ftc.teamcode.threaded.BotState.CarouselCommand;

@TeleOp(name = "TeleOp - RED", group = "TeleOp")
public class TeleOpRed extends LinearOpMode {

    // Red alliance uses tag 24
    private static final int BASKET_TAG_ID = CameraThread.TAG_RED_BASKET;

    private BotState state;
    private DriveThread driveThread;
    private ControlHubI2CThread controlHubI2C;
    private ExpansionHubI2CThread expansionHubI2C;
    private CarouselThread carouselThread;
    private ShooterThread shooterThread;
    private CameraThread cameraThread;
    private ElapsedTime runtime;

    // Button state tracking for edge detection
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevY = false;
    private boolean prevX = false;
    private boolean prevLBumper = false;
    private boolean prevRBumper = false;
    private boolean prevLBumper1 = false;
    private boolean prevRBumper1 = false;
    private boolean prevBack1 = false;

    // Shoot sequence

    @Override
    public void runOpMode() {
        runtime = new ElapsedTime();

        // Initialize state
        state = new BotState();

        // Initialize threads - pass RED basket tag ID to camera
        driveThread = new DriveThread(state, hardwareMap);
        controlHubI2C = new ControlHubI2CThread(state, hardwareMap);
        expansionHubI2C = new ExpansionHubI2CThread(state, hardwareMap);
        carouselThread = new CarouselThread(state, hardwareMap);
        shooterThread = new ShooterThread(state, hardwareMap);
        cameraThread = new CameraThread(state, hardwareMap, BASKET_TAG_ID);

        telemetry.addData("Alliance", "RED");
        telemetry.addData("Basket Tag", BASKET_TAG_ID);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Start threads
        driveThread.start();
        controlHubI2C.start();
        expansionHubI2C.start();
        carouselThread.start();
        shooterThread.start();
        cameraThread.start();

        // Main loop
        while (opModeIsActive()) {

            // Handle inputs
            handleDriveInput();
            handleCarouselInput();
            handleShooterInput();

            // Update shoot sequence
//            shootSequence.update();

            // Telemetry
            updateTelemetry();
        }

        // Clean up
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

    private void handleDriveInput() {
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x / 2.0;

        // L1 - Toggle auto-align
        if (gamepad1.left_bumper && !prevLBumper1) {
            state.toggleAutoAlign();
        }
        prevLBumper1 = gamepad1.left_bumper;

        // R1 - Update pose from AprilTag
        if (gamepad1.right_bumper && !prevRBumper1) {
            if (state.isBasketTagVisible() && state.getTagCalculatedPose() != null) {
                state.requestPoseUpdate();
            }
        }
        prevRBumper1 = gamepad1.right_bumper;

        // Share - Cancel path
        if (gamepad1.back && !prevBack1) {
            driveThread.getFollower().breakFollowing();
        }
        prevBack1 = gamepad1.back;

        // R2 - Intake forward, L2 - Intake reverse
        if (gamepad1.right_trigger > 0.1) {
            state.setIntakeForward(true);
            state.setIntakeReverse(false);
        } else if (gamepad1.left_trigger > 0.1) {
            state.setIntakeForward(false);
            state.setIntakeReverse(true);
        } else {
            state.setIntakeForward(false);
            state.setIntakeReverse(false);
        }

        // Set drive input (auto-align is handled in DriveThread)
        state.setDriveInput(forward, strafe, rotate);
    }

    private void handleCarouselInput() {
        // Circle - Show lights
        if (gamepad2.b && !prevB) {
            state.requestShowLights();
        }
        prevB = gamepad2.b;

        // Square - Kick (only if shooter ready)
        if (gamepad2.x && !prevX) {
            if (state.isShooterReady()) {
                state.requestKick();
            }
        }
        prevX = gamepad2.x;

        // D-pad Left - Rotate carousel left (one slot)
        if (gamepad2.dpad_left && !prevDpadLeft) {
            state.setCarouselCommand(CarouselCommand.ROTATE_LEFT);
        }
        prevDpadLeft = gamepad2.dpad_left;

        // D-pad Right - Rotate carousel right (one slot)
        if (gamepad2.dpad_right && !prevDpadRight) {
            state.setCarouselCommand(CarouselCommand.ROTATE_RIGHT);
        }
        prevDpadRight = gamepad2.dpad_right;

        // D-pad Up - Nudge carousel forward
        if (gamepad2.dpad_up && !prevDpadUp) {
            state.setCarouselCommand(CarouselCommand.NUDGE_FORWARD);
        }
        prevDpadUp = gamepad2.dpad_up;

        // D-pad Down - Nudge carousel backward
        if (gamepad2.dpad_down && !prevDpadDown) {
            state.setCarouselCommand(CarouselCommand.NUDGE_BACKWARD);
        }
        prevDpadDown = gamepad2.dpad_down;

        // Cross - Index (rotate empty to intake)
        if (gamepad2.a && !prevA) {
            state.setCarouselCommand(CarouselCommand.ROTATE_EMPTY_TO_INTAKE);
        }
        prevA = gamepad2.a;

        // Triangle - Shoot sequence
//        if (gamepad2.y && !prevY) {
//            shootSequence.start();
//        }
//        prevY = gamepad2.y;
    }

    private void handleShooterInput() {
        // L1 - Set velocity from distance
        if (gamepad2.left_bumper && !prevLBumper) {
            if (state.isBasketTagVisible()) {
                state.setAdjustedVelocity(state.getTagRange());
            }
        }
        prevLBumper = gamepad2.left_bumper;

        // R1 - Stop shooter
        if (gamepad2.right_bumper && !prevRBumper) {
            state.setShooterTargetVelocity(0);
        }
        prevRBumper = gamepad2.right_bumper;
    }

    // ========================= TELEMETRY =========================

    private void updateTelemetry() {
        telemetry.addData("Alliance", "RED (Tag %d)", BASKET_TAG_ID);
        telemetry.addData("Runtime", "%.1f s", runtime.seconds());

        telemetry.addLine("=== APRIL TAG ===");
        telemetry.addData("Camera State", state.getCameraState());
        telemetry.addData("Detections", state.getDetectionCount());
        telemetry.addData("Auto-Align", state.isAutoAlignEnabled() ? "ON" : "OFF");
        if (state.isBasketTagVisible()) {
            telemetry.addData("Basket Tag", "Bearing %.1f° | Range %.1f in",
                    state.getTagBearing(), state.getTagRange());
        } else {
            telemetry.addData("Basket Tag", "NOT VISIBLE");
        }
        if (state.hasDetectedShootOrder()) {
            BallColor[] order = state.getDetectedShootOrder();
            telemetry.addData("Shoot Order", "Tag %d: %s-%s-%s",
                    state.getShootOrderTagId(),
                    order[0].toString().charAt(0),
                    order[1].toString().charAt(0),
                    order[2].toString().charAt(0));
        } else {
            telemetry.addData("Shoot Order", "NOT DETECTED (using default)");
        }

        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Target", "%.0f deg/s", state.getShooterTargetVelocity());
        telemetry.addData("Current", "%.0f deg/s", state.getShooterCurrentVelocity());
        telemetry.addData("Ready", state.isShooterReady() ? "YES" : "NO");

        telemetry.addLine("=== DRIVE ===");
        telemetry.addData("Pose", "(%.1f, %.1f) %.1f°",
                state.getCurrentPose().getX(),
                state.getCurrentPose().getY(),
                Math.toDegrees(state.getCurrentPose().getHeading()));

        telemetry.addLine("=== CAROUSEL ===");
        telemetry.addData("Ball Count", state.getBallCount() + "/3");
        telemetry.addData("Settled", state.isCarouselSettled());
        telemetry.addData("Current Ticks", state.getCarouselCurrentTicks());
        telemetry.addData("Target Ticks", state.getCarouselTargetTicks());
        telemetry.addData("Error", state.getCarouselTargetTicks() - state.getCarouselCurrentTicks());
        telemetry.addData("TICKS_PER_SLOT", BotState.TICKS_PER_SLOT);

        BallColor[] positions = state.getAllPositions();
        telemetry.addData("INTAKE", positions[BotState.POS_INTAKE]);
        telemetry.addData("BACK_LEFT", positions[BotState.POS_BACK_LEFT]);
        telemetry.addData("BACK_RIGHT", positions[BotState.POS_BACK_RIGHT]);

        telemetry.addLine("=== INTAKE ===");
        telemetry.addData("Running", state.isIntakeRunning());
        telemetry.addData("Kicker", state.isKickerUp() ? "UP" : "DOWN");


        telemetry.update();
    }
}