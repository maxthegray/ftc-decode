package org.firstinspires.ftc.teamcode.threaded.Old.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.threaded.Old.CameraThread;
import org.firstinspires.ftc.teamcode.threaded.Old.CarouselController;
import org.firstinspires.ftc.teamcode.threaded.Old.ControlHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.Old.DriveThread;
import org.firstinspires.ftc.teamcode.threaded.Old.ExpansionHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.Old.MechanismThread;
import org.firstinspires.ftc.teamcode.threaded.Old.SensorState;
import org.firstinspires.ftc.teamcode.threaded.Old.ShootSequence;
import org.firstinspires.ftc.teamcode.threaded.Old.ShooterThread;

@TeleOp(name = "TeleOp - RED", group = "TeleOp")
public class TeleOpRed extends LinearOpMode {

    private static final int BASKET_TAG_ID = CameraThread.TAG_RED_BASKET;
    private static final double DEFAULT_VELOCITY = 150;

    public enum RobotMode {
        INTAKING,   // Auto-index ON
        SHOOTING    // Auto-index OFF
    }
    private RobotMode currentMode = RobotMode.INTAKING;

    private MechanismThread mechanismThread;
    private DriveThread driveThread;
    private ShooterThread shooterThread;
    private CameraThread cameraThread;
    private ControlHubI2CThread controlHubI2C;
    private ExpansionHubI2CThread expansionHubI2C;
    private SensorState sensorState;

    private ElapsedTime runtime;

    // Gamepad 1 edge detection
    private boolean prevLBumper1 = false;
    private boolean prevRBumper1 = false;

    // Gamepad 2 edge detection
    private boolean prevX2 = false;
    private boolean prevDpadLeft2 = false;
    private boolean prevDpadRight2 = false;
    private boolean prevDpadUp2 = false;
    private boolean prevDpadDown2 = false;
    private boolean prevLTrigger2 = false;  // Shoot green
    private boolean prevRTrigger2 = false;  // Shoot purple
    private boolean prevB2 = false;         // Show lights

    @Override
    public void runOpMode() {
        runtime = new ElapsedTime();
        sensorState = new SensorState(SensorState.Alliance.RED);

        // Initialize threads
        mechanismThread = new MechanismThread(hardwareMap);
        mechanismThread.setSensorState(sensorState);  // Needed for shooter-ready checks
        driveThread = new DriveThread(sensorState, hardwareMap);
        shooterThread = new ShooterThread(sensorState, hardwareMap);
        cameraThread = new CameraThread(sensorState, hardwareMap, BASKET_TAG_ID);
        controlHubI2C = new ControlHubI2CThread(sensorState, hardwareMap);
        expansionHubI2C = new ExpansionHubI2CThread(sensorState, hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        mechanismThread.start();
        driveThread.start();
        shooterThread.start();
        cameraThread.start();
        controlHubI2C.start();
        expansionHubI2C.start();

        while (opModeIsActive()) {
            mechanismThread.setBallPositions(sensorState.getAllPositions());

            handleDriverControls();
            handleGunnerControls();
            handleAutoModeSwitching();

            updateTelemetry();
        }

        // Cleanup
        mechanismThread.kill();
        sensorState.kill();
        joinAllThreads();
    }

    // ======================== DRIVER (Gamepad 1) ========================

    private void handleDriverControls() {
        // Mecanum Drive
        sensorState.setDriveInput(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x * 0.5
        );

        // LB — Toggle auto-align
        if (gamepad1.left_bumper && !prevLBumper1) sensorState.toggleAutoAlign();
        prevLBumper1 = gamepad1.left_bumper;

        // RB — Reset Pose from AprilTag
        if (gamepad1.right_bumper && !prevRBumper1) {
            if (sensorState.isBasketTagVisible()) sensorState.requestPoseUpdate();
        }
        prevRBumper1 = gamepad1.right_bumper;

        // Intake control — volatile state, not queued
        // Intake takes priority over shooter — they must never run simultaneously
        // Intake control — volatile state, not queued
// Intake takes priority over shooter — they must never run simultaneously
// Blocked while carousel is indexing to prevent jamming
        boolean carouselSettled = mechanismThread.isCarouselSettled();

        if (gamepad1.right_trigger > 0.1 && carouselSettled) {
            mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);
            sensorState.setShooterTargetVelocity(0);  // Kill shooter when intaking
            if (currentMode != RobotMode.INTAKING) switchMode(RobotMode.INTAKING);
        } else if (gamepad1.left_trigger > 0.1) {
            mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.OUT);
            sensorState.setShooterTargetVelocity(0);  // Kill shooter when reversing
        } else {
            mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
        }
    }

    // ======================== GUNNER (Gamepad 2) ========================

    private void handleGunnerControls() {
        boolean intakeActive = gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1;

        // Bumpers — Shooter spin-up / spin-down (blocked while intake is running)
        if (gamepad2.right_bumper && !intakeActive) {
            sensorState.setShooterTargetVelocity(DEFAULT_VELOCITY);
        } else if (gamepad2.left_bumper || intakeActive) {
            sensorState.setShooterTargetVelocity(0);
        }

        // X — Manual kick (whatever is at intake position)
        if (gamepad2.x && !prevX2) {
            switchMode(RobotMode.SHOOTING);
            ensureShooterSpinning();
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.KICK));
        }
        prevX2 = gamepad2.x;

        // Left Trigger — Shoot GREEN (find green, rotate to intake, kick)
        boolean ltPressed = gamepad2.left_trigger > 0.1;
        if (ltPressed && !prevLTrigger2) {
            switchMode(RobotMode.SHOOTING);
            ensureShooterSpinning();
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.SHOOT_SINGLE,
                            ShootSequence.BallColor.GREEN));
        }
        prevLTrigger2 = ltPressed;

        // Right Trigger — Shoot PURPLE (find purple, rotate to intake, kick)
        boolean rtPressed = gamepad2.right_trigger > 0.1;
        if (rtPressed && !prevRTrigger2) {
            switchMode(RobotMode.SHOOTING);
            ensureShooterSpinning();
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.SHOOT_SINGLE,
                            ShootSequence.BallColor.PURPLE));
        }
        prevRTrigger2 = rtPressed;

        // D-Pad Left / Right — Manual carousel rotation (1 full slot)
        if (gamepad2.dpad_left && !prevDpadLeft2) {
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.ROTATE_LEFT));
        }
        prevDpadLeft2 = gamepad2.dpad_left;

        if (gamepad2.dpad_right && !prevDpadRight2) {
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.ROTATE_RIGHT));
        }
        prevDpadRight2 = gamepad2.dpad_right;

        // D-Pad Up / Down — Nudge carousel (small adjustment)
        if (gamepad2.dpad_up && !prevDpadUp2) {
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.NUDGE,
                            CarouselController.NUDGE_TICKS));
        }
        prevDpadUp2 = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !prevDpadDown2) {
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.NUDGE,
                            -CarouselController.NUDGE_TICKS));
        }
        prevDpadDown2 = gamepad2.dpad_down;

        // Circle (B) — Show ball colors on lights
        if (gamepad2.circle && !prevB2) {
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.SHOW_LIGHTS));
        }
        prevB2 = gamepad2.circle;

        // Auto-scale shooter velocity from AprilTag distance (skip if intake active)
        if (!intakeActive && sensorState.getShooterTargetVelocity() > 0 && sensorState.isBasketTagVisible()) {
            sensorState.setVelocityFromDistance(sensorState.getTagRange());
        }
    }

    /**
     * Ensure the shooter is spinning at least at DEFAULT_VELOCITY.
     * If the tag is visible, the distance-based scaler will refine it next loop.
     * Does nothing if the intake is currently active (intake takes priority).
     */
    private void ensureShooterSpinning() {
        boolean intakeActive = gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1;
        if (!intakeActive && sensorState.getShooterTargetVelocity() <= 0) {
            sensorState.setShooterTargetVelocity(DEFAULT_VELOCITY);
        }
    }

    // ======================== AUTO MODE SWITCHING ========================

    private void handleAutoModeSwitching() {
        int ballCount = 0;
        for (ShootSequence.BallColor c : sensorState.getAllPositions()) {
            if (c != ShootSequence.BallColor.EMPTY) ballCount++;
        }

        if (ballCount >= 3 && currentMode == RobotMode.INTAKING) {
            switchMode(RobotMode.SHOOTING);
        }
    }

    private void switchMode(RobotMode mode) {
        currentMode = mode;
        boolean enableAuto = (mode == RobotMode.INTAKING);
        mechanismThread.enqueueCommand(
                new MechanismThread.Command(MechanismThread.Command.Type.SET_AUTO_INDEX, enableAuto));
    }

    // ======================== TELEMETRY ========================

    private void updateTelemetry() {
        telemetry.addData("Mode", currentMode);
        telemetry.addData("Balls", "%s|%s|%s",
                shortName(sensorState.getPositionColor(0)),
                shortName(sensorState.getPositionColor(1)),
                shortName(sensorState.getPositionColor(2)));

        // Shooter diagnostics
        double current = sensorState.getShooterCurrentVelocity();
        double target = sensorState.getShooterTargetVelocity();
        double error = Math.abs(current - target);
        boolean ready = sensorState.isShooterReady();
        telemetry.addData("Shooter", "%.0f / %.0f RPM (err %.0f) %s",
                current, target, error, ready ? "READY" : "NOT READY");


        // Carousel diagnostics
        boolean settled = mechanismThread.isCarouselSettled();
        telemetry.addData("Carousel", "curr: %d / target: %d | %s",
                mechanismThread.getCarouselCurrentTicks(),
                mechanismThread.getCarouselTargetTicks(),
                settled ? "SETTLED" : "MOVING");

        // Command acceptance diagnostics
        telemetry.addData("CmdReady", "idle=%b settled=%b",
                mechanismThread.isIdle(), settled);

        telemetry.update();
    }

    private String shortName(ShootSequence.BallColor c) {
        if (c == ShootSequence.BallColor.GREEN) return "G";
        if (c == ShootSequence.BallColor.PURPLE) return "P";
        return "-";
    }

    private void joinAllThreads() {
        try {
            mechanismThread.join(200);
            driveThread.join(200);
            shooterThread.join(200);
            cameraThread.join(200);
            controlHubI2C.join(200);
            expansionHubI2C.join(200);
        } catch (InterruptedException ignored) {}
    }
}