package org.firstinspires.ftc.teamcode.threaded;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.threaded.BotState.BallColor;

@TeleOp(name = "Threaded TeleOp", group = "TeleOp")
public class ThreadedTeleOp extends LinearOpMode {

    // Shared state
    private BotState state;

    // Threads
    private DriveThread driveThread;
    private CarouselThread carouselThread;
    private ShooterThread shooterThread;
    private CameraThread cameraThread;
    private ControlHubI2CThread controlHubI2CThread;
    private ExpansionHubI2CThread expansionHubI2CThread;

    // Shoot sequence manager
    private ShootSequenceManager shootSequence;

    // Alliance (change for red/blue)
    private static final int BASKET_TAG = CameraThread.TAG_RED_BASKET;

    // Button edge detection
    private boolean prevY = false;
    private boolean prevB = false;
    private boolean prevLB = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    @Override
    public void runOpMode() {

        // === INITIALIZATION ===
        state = new BotState();
        shootSequence = new ShootSequenceManager();

        driveThread = new DriveThread(state, hardwareMap);
        carouselThread = new CarouselThread(state, hardwareMap);
        shooterThread = new ShooterThread(state, hardwareMap);
        cameraThread = new CameraThread(state, hardwareMap, BASKET_TAG);
        controlHubI2CThread = new ControlHubI2CThread(state, hardwareMap);
        expansionHubI2CThread = new ExpansionHubI2CThread(state, hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine();
        telemetry.addLine("Y        = Shoot All 3");
        telemetry.addLine("DPad Up  = Shoot GREEN");
        telemetry.addLine("DPad Down= Shoot PURPLE");
        telemetry.addLine("B        = Abort");
        telemetry.addLine("LB       = Toggle Auto-Align");
        telemetry.update();

        waitForStart();

        // Start threads
        driveThread.start();
        carouselThread.start();
        shooterThread.start();
        cameraThread.start();
        controlHubI2CThread.start();
        expansionHubI2CThread.start();

        // === MAIN LOOP ===
        while (opModeIsActive()) {

            // --- DRIVE ---
            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double rotate = -gamepad1.right_stick_x;
            state.setDriveInput(forward, strafe, rotate);

            // --- AUTO-ALIGN (LB toggle) ---
            if (gamepad1.left_bumper && !prevLB) {
                state.toggleAutoAlign();
            }
            prevLB = gamepad1.left_bumper;

            // --- SHOOT ALL 3 (Y) ---
            if (gamepad1.y && !prevY && !shootSequence.isRunning()) {
                startFullSequence();
            }
            prevY = gamepad1.y;

            // --- SHOOT SINGLE GREEN (DPad Up) ---
            if (gamepad1.dpad_up && !prevDpadUp && !shootSequence.isRunning()) {
                shootSingleColor(BallColor.GREEN);
            }
            prevDpadUp = gamepad1.dpad_up;

            // --- SHOOT SINGLE PURPLE (DPad Down) ---
            if (gamepad1.dpad_down && !prevDpadDown && !shootSequence.isRunning()) {
                shootSingleColor(BallColor.PURPLE);
            }
            prevDpadDown = gamepad1.dpad_down;

            // --- ABORT (B) ---
            if (gamepad1.b && !prevB && shootSequence.isRunning()) {
                shootSequence.abort();
                state.setShooterTargetVelocity(0);
            }
            prevB = gamepad1.b;

            // Update sequence
            shootSequence.update(state);

            // Stop shooter when complete
            if (shootSequence.getState() == ShootSequenceManager.State.COMPLETE) {
                state.setShooterTargetVelocity(0);
            }

            // --- TELEMETRY ---
            telemetry.addLine("=== CAROUSEL ===");
            telemetry.addData("Positions", "[%s, %s, %s]",
                    state.getPositionColor(BotState.POS_INTAKE),
                    state.getPositionColor(BotState.POS_BACK_LEFT),
                    state.getPositionColor(BotState.POS_BACK_RIGHT));
            telemetry.addData("Current Ticks", state.getCarouselCurrentTicks());
            telemetry.addData("Target Ticks", state.getCarouselTargetTicks());
            telemetry.addData("Error", state.getCarouselTargetTicks() - state.getCarouselCurrentTicks());
            telemetry.addData("TICKS_PER_SLOT", BotState.TICKS_PER_SLOT);
            telemetry.addData("Settled", state.isCarouselSettled());

            telemetry.addLine();
            telemetry.addLine("=== AUTO-ALIGN ===");
            telemetry.addData("Enabled", state.isAutoAlignEnabled() ? "ON" : "OFF");
            if (state.isBasketTagVisible()) {
                telemetry.addData("Bearing", "%.1f°", state.getTagBearing());
                telemetry.addData("Range", "%.1f in", state.getTagRange());
            } else {
                telemetry.addData("Tag", "NOT VISIBLE");
            }

            telemetry.addLine();
            telemetry.addLine("=== SHOOT SEQUENCE ===");
            telemetry.addLine(shootSequence.getDebugString());
            if (shootSequence.getRotationPlan() != null) {
                telemetry.addLine(shootSequence.getPlanDebugString());
            }

            telemetry.update();
        }

        // === CLEANUP ===
        state.endThreads();
    }

    /**
     * Start full 3-ball sequence.
     */
    private void startFullSequence() {
        // Get target order from AprilTag or default
        BallColor[] targetOrder = state.getDetectedShootOrder();
        if (targetOrder == null) {
            targetOrder = new BallColor[] {
                    BallColor.GREEN,
                    BallColor.PURPLE,
                    BallColor.PURPLE
            };
        }

        // Set shooter velocity
        setShooterVelocity();

        // Start
        shootSequence.start(state, targetOrder);
    }

    /**
     * Shoot a single ball of specified color.
     */
    private void shootSingleColor(BallColor color) {
        // Set shooter velocity
        setShooterVelocity();

        // Start single shot
        shootSequence.shootSingle(state, color);
    }

    /**
     * Set shooter velocity based on tag distance or default.
     */
    private void setShooterVelocity() {
        if (state.isBasketTagVisible()) {
            state.setAdjustedVelocity(state.getTagRange());
        } else {
            state.setShooterTargetVelocity(210);
        }
    }
}