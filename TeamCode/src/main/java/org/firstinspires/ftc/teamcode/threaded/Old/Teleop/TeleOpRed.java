package org.firstinspires.ftc.teamcode.threaded.Old.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.threaded.Old.BotState;
import org.firstinspires.ftc.teamcode.threaded.Old.BotState.BallColor;
import org.firstinspires.ftc.teamcode.threaded.Old.CameraThread;
import org.firstinspires.ftc.teamcode.threaded.Old.CarouselThread;
import org.firstinspires.ftc.teamcode.threaded.Old.ControlHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.Old.DriveThread;
import org.firstinspires.ftc.teamcode.threaded.Old.ExpansionHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.Old.ShootSequenceManager;
import org.firstinspires.ftc.teamcode.threaded.Old.ShooterThread;

@TeleOp(name = "TeleOp - RED", group = "TeleOp")
public class TeleOpRed extends LinearOpMode {

    // Red alliance uses tag 24
    private static final int BASKET_TAG_ID = CameraThread.TAG_RED_BASKET;

    // Default shoot order if no AprilTag detected
    private static final BallColor[] DEFAULT_SHOOT_ORDER = {
            BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE
    };

    // Default velocity if no tag visible
    private static final double DEFAULT_VELOCITY = 210.0;

    // Trigger threshold for edge detection
    private static final float TRIGGER_THRESHOLD = 0.5f;

    // ========================= ROBOT MODE STATE MACHINE =========================
    public enum RobotMode {
        INTAKING,   // Auto-indexing ON
        SHOOTING    // Auto-indexing OFF
    }
    private RobotMode currentMode = RobotMode.INTAKING;

    private BotState state;
    private DriveThread driveThread;
    private ControlHubI2CThread controlHubI2C;
    private ExpansionHubI2CThread expansionHubI2C;
    private CarouselThread carouselThread;
    private ShooterThread shooterThread;
    private CameraThread cameraThread;
    private ElapsedTime runtime;

    // Shoot sequence manager
    private ShootSequenceManager shootSequence;

    // Button state tracking for edge detection
    private boolean prevB = false;
    private boolean prevY = false;
    private boolean prevX = false;
    private boolean prevLBumper = false;
    private boolean prevRBumper = false;
    private boolean prevLBumper1 = false;
    private boolean prevRBumper1 = false;
    private boolean prevBack1 = false;

    // Trigger state tracking for edge detection (gamepad2)
    private boolean prevLTrigger2 = false;
    private boolean prevRTrigger2 = false;

    // D-pad state tracking for manual carousel control (gamepad2)
    private boolean prevDpadUp2 = false;
    private boolean prevDpadDown2 = false;
    private boolean prevDpadLeft2 = false;
    private boolean prevDpadRight2 = false;

    // Track previous carousel full state for edge detection
    private boolean wasCarouselFull = false;

    @Override
    public void runOpMode() {
        runtime = new ElapsedTime();

        // Initialize state
        state = new BotState();

        // Initialize shoot sequence manager
        shootSequence = new ShootSequenceManager();

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
            handleShooterInput();
            handleModeTransitions();

            // Update shoot sequence
            shootSequence.update(state);

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

    private void handleShooterInput() {
        // Circle/B - Show lights
        if (gamepad2.b && !prevB) {
            state.requestShowLights();
        }
        prevB = gamepad2.b;

        // Square/X - Kick (only if shooter ready)
        if (gamepad2.x && !prevX) {
            if (state.isShooterReady() && state.isCarouselSettled()) {
                state.requestKick();
            }
        }
        prevX = gamepad2.x;

        // Left Trigger (L2) - Shoot green
        boolean lTriggerPressed = gamepad2.left_trigger > TRIGGER_THRESHOLD;
        if (lTriggerPressed && !prevLTrigger2) {
            if (state.hasColor(BallColor.GREEN) && !shootSequence.isRunning()) {
                switchToShootingMode();
                startShooterMotor();
                shootSequence.shootSingle(state, BallColor.GREEN);
            }
        }
        prevLTrigger2 = lTriggerPressed;

        // Right Trigger (R2) - Shoot purple
        boolean rTriggerPressed = gamepad2.right_trigger > TRIGGER_THRESHOLD;
        if (rTriggerPressed && !prevRTrigger2) {
            if (state.hasColor(BallColor.PURPLE) && !shootSequence.isRunning()) {
                switchToShootingMode();
                startShooterMotor();
                shootSequence.shootSingle(state, BallColor.PURPLE);
            }
        }
        prevRTrigger2 = rTriggerPressed;

        // Triangle/Y - Shoot full sorted sequence
        if (gamepad2.y && !prevY) {
            if (state.getBallCount() >= 3) {
                switchToShootingMode();
                startShooterMotor();
                // Use detected shoot order or default
                BallColor[] order = state.hasDetectedShootOrder()
                        ? state.getDetectedShootOrder()
                        : DEFAULT_SHOOT_ORDER;
                shootSequence.start(state, order);
            }
        }
        prevY = gamepad2.y;

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

        // D-pad Up/Down - Micro-adjust carousel by NUDGE_TICKS
        boolean canIssueCarouselCommand = state.isCarouselSettled() &&
                state.getCarouselCommand() == BotState.CarouselCommand.NONE;

        if (gamepad2.dpad_up && !prevDpadUp2 && canIssueCarouselCommand) {
            state.setCarouselCommand(BotState.CarouselCommand.NUDGE_FORWARD);
        }
        prevDpadUp2 = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !prevDpadDown2 && canIssueCarouselCommand) {
            state.setCarouselCommand(BotState.CarouselCommand.NUDGE_BACKWARD);
        }
        prevDpadDown2 = gamepad2.dpad_down;

        // D-pad Left/Right - Rotate carousel by 1 slot
        if (gamepad2.dpad_left && !prevDpadLeft2 && canIssueCarouselCommand) {
            state.setCarouselCommand(BotState.CarouselCommand.ROTATE_LEFT);
        }
        prevDpadLeft2 = gamepad2.dpad_left;

        if (gamepad2.dpad_right && !prevDpadRight2 && canIssueCarouselCommand) {
            state.setCarouselCommand(BotState.CarouselCommand.ROTATE_RIGHT);
        }
        prevDpadRight2 = gamepad2.dpad_right;
    }

    private void handleModeTransitions() {
        // Transition to SHOOTING when carousel becomes full
        boolean isCarouselFull = state.isFull();
        if (isCarouselFull && !wasCarouselFull) {
            switchToShootingMode();
        }
        wasCarouselFull = isCarouselFull;

        // Transition to INTAKING when:
        // 1. Carousel becomes empty (but NOT while a shoot sequence is running)
        // 2. Driver triggers intake (R2)
        if (currentMode == RobotMode.SHOOTING) {
            // Don't auto-switch to intaking while shoot sequence is running
            // (sensors may momentarily read empty during carousel rotation)
            if (state.isEmpty() && !shootSequence.isRunning()) {
                switchToIntakingMode();
            } else if (gamepad1.right_trigger > 0.1) {
                switchToIntakingMode();
            }
        }
    }

    private void switchToShootingMode() {
        currentMode = RobotMode.SHOOTING;
        state.setAutoIndexEnabled(false);
    }

    private void switchToIntakingMode() {
        currentMode = RobotMode.INTAKING;
        state.setAutoIndexEnabled(true);
        // Abort any running shoot sequence
        if (shootSequence.isRunning()) {
            shootSequence.abort();
        }
    }

    /**
     * Start the shooter motor - use distance-based velocity if tag visible, otherwise default
     */
    private void startShooterMotor() {
        if (state.isBasketTagVisible()) {
            state.setAdjustedVelocity(state.getTagRange());
        } else {
            state.setShooterTargetVelocity(DEFAULT_VELOCITY);
        }
    }

    // ========================= TELEMETRY =========================

    private void updateTelemetry() {
        telemetry.addData("Alliance", "RED (Tag %d)", BASKET_TAG_ID);
        telemetry.addData("Runtime", "%.1f s", runtime.seconds());

        // Current mode prominently displayed
        telemetry.addLine("=== MODE ===");
        telemetry.addData("Current Mode", currentMode == RobotMode.INTAKING ? ">>> INTAKING <<<" : ">>> SHOOTING <<<");
        telemetry.addData("Auto-Index", state.isAutoIndexEnabled() ? "ON" : "OFF");

        telemetry.addLine("=== APRIL TAG ===");
        telemetry.addData("Auto-Align", state.isAutoAlignEnabled() ? "ON" : "OFF");
        if (state.isBasketTagVisible()) {
            telemetry.addData("Range", "%.1f in", state.getTagRange());
            telemetry.addData("Raw Bearing", "%.1f°", state.getTagBearing());
            telemetry.addData("Yaw", "%.1f°", state.getTagYaw());
            telemetry.addData("Target Bearing", "%.1f°", state.getTargetBearing());
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
        }

        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Target", "%.0f deg/s", state.getShooterTargetVelocity());
        telemetry.addData("Current", "%.0f deg/s", state.getShooterCurrentVelocity());
        telemetry.addData("Ready", state.isShooterReady() ? "YES" : "NO");

        // Shoot sequence status
        if (shootSequence.isRunning()) {
            telemetry.addData("Shoot Seq", shootSequence.getDebugString());
        }

        telemetry.addLine("=== DRIVE ===");
        telemetry.addData("Pose", "(%.1f, %.1f) %.1f°",
                state.getCurrentPose().getX(),
                state.getCurrentPose().getY(),
                Math.toDegrees(state.getCurrentPose().getHeading()));

        telemetry.addLine("=== CAROUSEL ===");
        telemetry.addData("Settled", state.isCarouselSettled());
        telemetry.addData("Target Ticks", state.getCarouselTargetTicks());
        telemetry.addData("Current Ticks", state.getCarouselCurrentTicks());

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