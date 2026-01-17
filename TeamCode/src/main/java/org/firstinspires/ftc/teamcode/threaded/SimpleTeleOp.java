package org.firstinspires.ftc.teamcode.threaded;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.threaded.BotState.BallColor;
import org.firstinspires.ftc.teamcode.threaded.BotState.CarouselCommand;

@TeleOp(name = "Threaded TeleOp", group = "TeleOp")
public class SimpleTeleOp extends LinearOpMode {

    private BotState state;
    private DriveThread driveThread;
    private ControlHubI2CThread controlHubI2C;
    private ExpansionHubI2CThread expansionHubI2C;
    private CarouselThread carouselThread;
    private ShooterThread shooterThread;
    private CameraThread cameraThread;
    private ElapsedTime runtime;

    // Button state tracking for debounce
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevLBumper = false;
    private boolean prevRBumper = false;
    private boolean prevLBumper1 = false;

    // Shooter velocity increments
    private static final double VELOCITY_INCREMENT_SMALL = 1.0;
    private static final double VELOCITY_INCREMENT_LARGE = 5.0;

    // Auto-align constants
    private static final double ALIGN_POWER = 0.15;
    private static final double BEARING_TOLERANCE = 1.0;

    @Override
    public void runOpMode() {
        runtime = new ElapsedTime();

        // Initialize state
        state = new BotState();

        // Initialize threads
        driveThread = new DriveThread(state, hardwareMap);
        driveThread.initialize();

        controlHubI2C = new ControlHubI2CThread(state, hardwareMap);
        expansionHubI2C = new ExpansionHubI2CThread(state, hardwareMap);
        carouselThread = new CarouselThread(state, hardwareMap);
        shooterThread = new ShooterThread(state, hardwareMap);
        cameraThread = new CameraThread(state, hardwareMap);

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

        // Auto-align toggle - Left Bumper on gamepad1
        if (gamepad1.left_bumper && !prevLBumper1) {
            state.toggleAutoAlign();
        }
        prevLBumper1 = gamepad1.left_bumper;

        // Apply auto-align rotation if enabled and tag visible
        if (state.isAutoAlignEnabled() && state.isBasketTagVisible()) {
            double bearing = state.getTagBearing();
            if (Math.abs(bearing) > BEARING_TOLERANCE) {
                rotate = (bearing > 0) ? -ALIGN_POWER : ALIGN_POWER;
            } else {
                rotate = 0;
            }
        }

        state.setDriveInput(forward, strafe, rotate);

        // Show lights - Square on driver gamepad (rising edge)
        if (gamepad1.x && !prevX) {
            state.requestShowLights();
        }
        prevX = gamepad1.x;
    }

    private void handleCarouselInput() {
        // Intake - Triggers on gamepad2
        if (gamepad2.right_trigger > 0.1) {
            state.setIntakeForward(true);
            state.setIntakeReverse(false);
        } else if (gamepad2.left_trigger > 0.1) {
            state.setIntakeForward(false);
            state.setIntakeReverse(true);
        } else {
            state.setIntakeForward(false);
            state.setIntakeReverse(false);
        }

        // Kick - B on gamepad2 (rising edge)
        if (gamepad2.b && !prevB) {
            state.requestKick();
        }
        prevB = gamepad2.b;

        // Carousel rotate left - D-pad left (rising edge)
        if (gamepad2.dpad_left && !prevDpadLeft) {
            state.setCarouselCommand(CarouselCommand.ROTATE_LEFT);
        }
        prevDpadLeft = gamepad2.dpad_left;

        // Carousel rotate right - D-pad right (rising edge)
        if (gamepad2.dpad_right && !prevDpadRight) {
            state.setCarouselCommand(CarouselCommand.ROTATE_RIGHT);
        }
        prevDpadRight = gamepad2.dpad_right;

        // Index (rotate empty to intake) - A on gamepad2 (rising edge)
        if (gamepad2.a && !prevA) {
            state.setCarouselCommand(CarouselCommand.ROTATE_EMPTY_TO_INTAKE);
        }
        prevA = gamepad2.a;
    }

    private void handleShooterInput() {
        // Bumpers - coarse adjust (±5 deg/s)
        if (gamepad2.left_bumper && !prevLBumper) {
            state.adjustShooterVelocity(-VELOCITY_INCREMENT_LARGE);
        }
        prevLBumper = gamepad2.left_bumper;

        if (gamepad2.right_bumper && !prevRBumper) {
            state.adjustShooterVelocity(VELOCITY_INCREMENT_LARGE);
        }
        prevRBumper = gamepad2.right_bumper;

        // D-pad up/down - fine adjust (±1 deg/s)
        if (gamepad2.dpad_up && !prevDpadUp) {
            state.adjustShooterVelocity(VELOCITY_INCREMENT_SMALL);
        }
        prevDpadUp = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !prevDpadDown) {
            state.adjustShooterVelocity(-VELOCITY_INCREMENT_SMALL);
        }
        prevDpadDown = gamepad2.dpad_down;

        // X - shooter off
        if (gamepad2.x) {
            state.setShooterTargetVelocity(0);
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Runtime", "%.1f s", runtime.seconds());

        telemetry.addLine("=== APRIL TAG ===");
        telemetry.addData("Auto-Align", state.isAutoAlignEnabled() ? "ON" : "OFF");
        if (state.isBasketTagVisible()) {
            telemetry.addData("Tag", "ID %d | Bearing %.1f° | Range %.1f in",
                    state.getTagId(), state.getTagBearing(), state.getTagRange());
        } else {
            telemetry.addData("Tag", "NOT VISIBLE");
        }

        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Target", "%.0f deg/s", state.getShooterTargetVelocity());
        telemetry.addData("Current", "%.0f deg/s", state.getShooterCurrentVelocity());
        telemetry.addData("Can Run", state.canShooterRun() ? "YES" : "NO (intake)");

        telemetry.addLine("=== DRIVE ===");
        telemetry.addData("Pose", state.getCurrentPose());

        telemetry.addLine("=== CAROUSEL ===");
        telemetry.addData("Ball Count", state.getBallCount() + "/3");
        telemetry.addData("Settled", state.isCarouselSettled());

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