package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.threaded.CameraThread;
import org.firstinspires.ftc.teamcode.threaded.CarouselController;
import org.firstinspires.ftc.teamcode.threaded.ControlHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.DriveThread;
import org.firstinspires.ftc.teamcode.threaded.ExpansionHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.MechanismThread;
import org.firstinspires.ftc.teamcode.threaded.SensorState;
import org.firstinspires.ftc.teamcode.threaded.ShootSequence;
import org.firstinspires.ftc.teamcode.threaded.ShooterThread;

/**
 * ══════════════════════════════════════════════════════════════════════════════
 *  AUTO-ALIGN PID TUNER — mirrors TeleOp loop exactly
 * ══════════════════════════════════════════════════════════════════════════════
 *
 *  Main loop mirrors TeleOpBlue/Red exactly (handleDriverControls,
 *  handleGunnerControls, handleAutoModeSwitching, same telemetry volume)
 *  so the CPU load and loop rate match real match conditions.
 *
 *  PID tuning controls are layered on top via A/B on GP1 and d-pad on GP2.
 *  These don't conflict with the TeleOp controls because:
 *    - GP1 A/B are unused in TeleOp driver controls
 *    - GP2 d-pad left/right ARE used for carousel in TeleOp (and here too),
 *      so tuning adjustments use GP2 d-pad up/down + A/B instead.
 *
 *  ── TUNING OVERLAY ──────────────────────────────────────────────────────────
 *
 *  GP1 A             — Toggle auto-align ON / OFF (with PID reset)
 *  GP1 B             — Toggle RAW / INTERPOLATED bearing
 *  GP2 A             — Cycle selected term (P → I → D → DEADBAND)
 *  GP2 B             — Increase selected term
 *  GP2 Y (hold)      — Also toggles auto-align (same as TeleOp)
 *  GP2 RB + B        — 10× step (coarse)
 *  GP2 LB + B        — 0.1× step (fine)
 *
 * ══════════════════════════════════════════════════════════════════════════════
 */
@TeleOp(name = "Auto-Align Tuner (Threaded)", group = "Tuning")
public class HeadingPIDTuner extends LinearOpMode {

    private static final int BASKET_TAG_ID = CameraThread.TAG_RED_BASKET;
    private static final double DEFAULT_VELOCITY = 150;
    private static final double STICK_EXPONENT = 2.0;

    // ── Robot mode (mirrors TeleOp) ───────────────────────────────────────
    public enum RobotMode { INTAKING, SHOOTING }
    private RobotMode currentMode = RobotMode.INTAKING;

    // ── Threads ───────────────────────────────────────────────────────────
    private SensorState sensorState;
    private DriveThread driveThread;
    private  CameraThread cameraThread;
    private MechanismThread mechanismThread;
    private ShooterThread shooterThread;
    private ControlHubI2CThread controlHubI2C;
    private ExpansionHubI2CThread expansionHubI2C;

    private ElapsedTime runtime;

    // ── GP1 edge detection (mirrors TeleOp + tuning extras) ───────────────
    private boolean prevLBumper1 = false;
    private boolean prevRBumper1 = false;
    private boolean prevA1 = false;
    private boolean prevB1 = false;

    // ── GP2 edge detection (mirrors TeleOp + tuning extras) ───────────────
    private boolean prevX2 = false;
    private boolean prevDpadLeft2 = false;
    private boolean prevDpadRight2 = false;
    private boolean prevLTrigger2 = false;
    private boolean prevRTrigger2 = false;
    private boolean prevB2 = false;
    private boolean prevY2 = false;
    private boolean prevA2 = false;

    // ── PID tuning state ──────────────────────────────────────────────────
    private enum SelectedTerm { P, I, D, DEADBAND, D_ALPHA }
    private SelectedTerm selected = SelectedTerm.P;

    @Override
    public void runOpMode() {
        runtime = new ElapsedTime();
        sensorState = new SensorState(SensorState.Alliance.RED);

        mechanismThread = new MechanismThread(hardwareMap);
        mechanismThread.setSensorState(sensorState);
        mechanismThread.setSkipKickback(true);
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

            // ── Identical TeleOp processing ───────────────────────────────
            handleDriverControls();
            handleGunnerControls();
            handleAutoModeSwitching();

            // ── Tuning overlay (GP1 A/B, GP2 A/B) ────────────────────────
            handleTuningControls();

            updateTelemetry();
        }

        mechanismThread.kill();
        sensorState.kill();
        joinAllThreads();
    }

    // ══════════════════════════════════════════════════════════════════════
    //  DRIVER CONTROLS — copied from TeleOpBlue/Red
    // ══════════════════════════════════════════════════════════════════════

    private double applyCurve(double input) {
        return Math.copySign(Math.pow(Math.abs(input), STICK_EXPONENT), input);
    }

    private void handleDriverControls() {
        sensorState.setDriveInput(
                applyCurve(-gamepad1.left_stick_y),
                applyCurve(-gamepad1.left_stick_x),
                applyCurve(-gamepad1.right_stick_x) * 0.75
        );

        // LB — Toggle auto-align (same as TeleOp)
        if (gamepad1.left_bumper && !prevLBumper1) {
            sensorState.toggleAutoAlign();
            if (sensorState.isAutoAlignEnabled()) {
                driveThread.requestPidReset();
            }
        }
        prevLBumper1 = gamepad1.left_bumper;

        // RB — Reset Pose from AprilTag
        if (gamepad1.right_bumper && !prevRBumper1) {
            if (sensorState.isBasketTagVisible()) sensorState.requestPoseUpdate();
        }
        prevRBumper1 = gamepad1.right_bumper;

        // Intake control
        boolean carouselSettled = mechanismThread.isCarouselSettled();

        if (gamepad1.right_trigger > 0.1 && carouselSettled) {
            mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);
            sensorState.setShooterTargetVelocity(0);
            if (currentMode != RobotMode.INTAKING) switchMode(RobotMode.INTAKING);
        } else if (gamepad1.left_trigger > 0.1) {
            mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.OUT);
            sensorState.setShooterTargetVelocity(0);
        } else {
            mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
        }
    }

    // ══════════════════════════════════════════════════════════════════════
    //  GUNNER CONTROLS — copied from TeleOpBlue/Red
    // ══════════════════════════════════════════════════════════════════════

    private void handleGunnerControls() {
        boolean intakeActive = gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1;

        if (gamepad2.right_bumper && !intakeActive) {
            sensorState.setShooterTargetVelocity(DEFAULT_VELOCITY);
        } else if (gamepad2.left_bumper || intakeActive) {
            sensorState.setShooterTargetVelocity(0);
        }

        if (gamepad2.x && !prevX2) {
            switchMode(RobotMode.SHOOTING);
            ensureShooterSpinning();
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.KICK));
        }
        prevX2 = gamepad2.x;

        boolean ltPressed = gamepad2.left_trigger > 0.1;
        if (ltPressed && !prevLTrigger2) {
            switchMode(RobotMode.SHOOTING);
            ensureShooterSpinning();
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.SHOOT_SINGLE,
                            ShootSequence.BallColor.GREEN));
        }
        prevLTrigger2 = ltPressed;

        boolean rtPressed = gamepad2.right_trigger > 0.1;
        if (rtPressed && !prevRTrigger2) {
            switchMode(RobotMode.SHOOTING);
            ensureShooterSpinning();
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.SHOOT_SINGLE,
                            ShootSequence.BallColor.PURPLE));
        }
        prevRTrigger2 = rtPressed;

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

        // Left stick Y — Analog nudge carousel
        double nudgeInput = applyCurve(-gamepad2.left_stick_y);
        if (Math.abs(nudgeInput) > 0.05) {
            int nudgeTicks = (int)(nudgeInput * CarouselController.NUDGE_TICKS);
            mechanismThread.setNudgeRequest(nudgeTicks);
        } else {
            mechanismThread.setNudgeRequest(0);
        }

        // Triangle (Y) — Toggle auto-align (same as TeleOp)
        if (gamepad2.y && !prevY2) {
            sensorState.toggleAutoAlign();
            if (sensorState.isAutoAlignEnabled()) {
                driveThread.requestPidReset();
            }
        }
        prevY2 = gamepad2.y;

        if (gamepad2.circle && !prevB2) {
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.SHOW_LIGHTS));
        }
        prevB2 = gamepad2.circle;

        if (!intakeActive && sensorState.getShooterTargetVelocity() > 0) {
            if (sensorState.isBasketTagVisible()) {
                sensorState.setVelocityFromDistance(sensorState.getTagRange());
            } else {
                sensorState.setVelocityFromDistance(sensorState.getOdometryDistanceToBasket());
            }
        }
    }

    private void ensureShooterSpinning() {
        boolean intakeActive = gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1;
        if (!intakeActive && sensorState.getShooterTargetVelocity() <= 0) {
            sensorState.setShooterTargetVelocity(DEFAULT_VELOCITY);
        }
    }

    // ══════════════════════════════════════════════════════════════════════
    //  AUTO MODE SWITCHING — copied from TeleOpBlue/Red
    // ══════════════════════════════════════════════════════════════════════

    private void handleAutoModeSwitching() {
        int ballCount = 0;
        for (ShootSequence.BallColor c : sensorState.getAllPositions()) {
            if (c != ShootSequence.BallColor.EMPTY) ballCount++;
        }

        if (ballCount >= 3 && currentMode == RobotMode.INTAKING) {
            switchMode(RobotMode.SHOOTING);
            boolean intakeActive = gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1;
            if (!intakeActive) {
                sensorState.setShooterTargetVelocity(DEFAULT_VELOCITY);
            }
        }
    }

    private void switchMode(RobotMode mode) {
        currentMode = mode;
        boolean enableAuto = (mode == RobotMode.INTAKING);
        mechanismThread.enqueueCommand(
                new MechanismThread.Command(MechanismThread.Command.Type.SET_AUTO_INDEX, enableAuto));
    }

    // ══════════════════════════════════════════════════════════════════════
    //  TUNING OVERLAY — extra controls on GP1 A/B, GP2 A/B
    // ══════════════════════════════════════════════════════════════════════

    private void handleTuningControls() {
        // GP1 A — Toggle auto-align with PID reset (dedicated tuning button)
        if (gamepad1.a && !prevA1) {
            sensorState.toggleAutoAlign();
            if (sensorState.isAutoAlignEnabled()) {
                driveThread.requestPidReset();
            }
        }
        prevA1 = gamepad1.a;

        // GP1 B — Toggle raw / interpolated bearing
        if (gamepad1.b && !prevB1) {
            driveThread.setUseRawBearing(!driveThread.getUseRawBearing());
            driveThread.requestPidReset();
        }
        prevB1 = gamepad1.b;

        // GP2 A — Cycle selected PID term
        if (gamepad2.a && !prevA2) {
            selected = nextTerm(selected);
        }
        prevA2 = gamepad2.a;

        // GP2 dpad_up — Increase selected term
        // GP2 dpad_down — Decrease selected term
        // (dpad_left/right are used by carousel, so we use up/down for tuning)
        double step = getStep();
        if (gamepad2.right_bumper) step *= 10.0;
        if (gamepad2.left_bumper) step *= 0.1;

        if (gamepad2.dpad_up) adjustSelected(step * 0.05);   // held = continuous
        if (gamepad2.dpad_down) adjustSelected(-step * 0.05); // held = continuous
    }

    // ══════════════════════════════════════════════════════════════════════
    //  TELEMETRY — matches TeleOp volume, with PID info added
    // ══════════════════════════════════════════════════════════════════════

    private void updateTelemetry() {
        telemetry.addData("Mode", currentMode);
        telemetry.addData("Balls", "%s|%s|%s",
                shortName(sensorState.getPositionColor(0)),
                shortName(sensorState.getPositionColor(1)),
                shortName(sensorState.getPositionColor(2)));

        double current = sensorState.getShooterCurrentVelocity();
        double target = sensorState.getShooterTargetVelocity();
        double error = Math.abs(current - target);
        boolean ready = sensorState.isShooterReady();
        telemetry.addData("Shooter", "%.0f / %.0f RPM (err %.0f) %s",
                current, target, error, ready ? "READY" : "NOT READY");

        boolean settled = mechanismThread.isCarouselSettled();
        telemetry.addData("Carousel", "curr: %d / target: %d | %s",
                mechanismThread.getCarouselCurrentTicks(),
                mechanismThread.getCarouselTargetTicks(),
                settled ? "SETTLED" : "MOVING");

        // PID tuning info (replaces CmdReady line — same telemetry item count)
        telemetry.addData("Align", "%s | %s | PID=%.3f",
                sensorState.isAutoAlignEnabled() ? "ON" : "off",
                driveThread.getUseRawBearing() ? "RAW" : "INTERP",
                driveThread.getDiagPidOutput());
        telemetry.addData("Tune",
                "%s P=%.4f I=%.4f D=%.4f DB=%.1f A=%.2f",
                selected.name(),
                SensorState.ALIGN_P, SensorState.ALIGN_I,
                SensorState.ALIGN_D, SensorState.ALIGN_DEADBAND,
                SensorState.ALIGN_D_ALPHA);

        telemetry.update();
    }

    private String shortName(ShootSequence.BallColor c) {
        if (c == ShootSequence.BallColor.GREEN) return "G";
        if (c == ShootSequence.BallColor.PURPLE) return "P";
        return "-";
    }

    // ── Tuning helpers ────────────────────────────────────────────────────

    private double getStep() {
        switch (selected) {
            case P: return 0.001;
            case I: return 0.0005;
            case D: return 0.0002;
            case DEADBAND: return 0.1;
            case D_ALPHA: return 0.05;
            default: return 0.001;
        }
    }

    private void adjustSelected(double delta) {
        switch (selected) {
            case P: SensorState.ALIGN_P = Math.max(0, SensorState.ALIGN_P + delta); break;
            case I: SensorState.ALIGN_I = Math.max(0, SensorState.ALIGN_I + delta); break;
            case D: SensorState.ALIGN_D = Math.max(0, SensorState.ALIGN_D + delta); break;
            case DEADBAND: SensorState.ALIGN_DEADBAND = Math.max(0, SensorState.ALIGN_DEADBAND + delta); break;
            case D_ALPHA: SensorState.ALIGN_D_ALPHA = Math.max(0, Math.min(1.0, SensorState.ALIGN_D_ALPHA + delta)); break;
        }
    }

    private SelectedTerm nextTerm(SelectedTerm t) {
        switch (t) {
            case P: return SelectedTerm.I;
            case I: return SelectedTerm.D;
            case D: return SelectedTerm.DEADBAND;
            case DEADBAND: return SelectedTerm.D_ALPHA;
            case D_ALPHA: return SelectedTerm.P;
            default: return SelectedTerm.P;
        }
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
