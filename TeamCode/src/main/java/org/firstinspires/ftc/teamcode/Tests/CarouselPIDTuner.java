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
 *  CAROUSEL PID TUNER — mirrors TeleOp loop exactly
 * ══════════════════════════════════════════════════════════════════════════════
 *
 *  Runs through the real MechanismThread → CarouselController pipeline,
 *  with all TeleOp threads active for identical CPU contention.
 *
 *  CarouselController's gains (KP, KI, KD, MAX_POWER, TOLERANCE) are
 *  public static fields, so changes here take effect immediately.
 *
 *  ── TUNING CONTROLS (on top of normal TeleOp controls) ─────────────────────
 *
 *  GP1 A              — Command ONE slot forward  (+1)
 *  GP1 B              — Command ONE slot backward (-1)
 *  GP1 X              — Command THREE slots forward
 *  GP2 A              — Cycle selected param (KP → KI → KD → TOL → MAX)
 *  GP2 dpad_up/down   — Increase / decrease selected param
 *  GP2 RB (hold)      — 10× step (coarse)
 *  GP2 LB (hold)      — 0.1× step (fine)
 *
 *  All other GP1/GP2 controls work as normal TeleOp (intake, shooter, etc.)
 *
 * ══════════════════════════════════════════════════════════════════════════════
 */
@TeleOp(name = "Carousel PID Tuner", group = "Tuning")
public class CarouselPIDTuner extends LinearOpMode {

    private static final int BASKET_TAG_ID = CameraThread.TAG_RED_BASKET;
    private static final double DEFAULT_VELOCITY = 150;
    private static final double STICK_EXPONENT = 2.0;

    // ── Robot mode (mirrors TeleOp) ───────────────────────────────────────
    public enum RobotMode { INTAKING, SHOOTING }
    private RobotMode currentMode = RobotMode.INTAKING;

    // ── Threads ───────────────────────────────────────────────────────────
    private SensorState sensorState;
    private DriveThread driveThread;
    private CameraThread cameraThread;
    private MechanismThread mechanismThread;
    private ShooterThread shooterThread;
    private ControlHubI2CThread controlHubI2C;
    private ExpansionHubI2CThread expansionHubI2C;

    private ElapsedTime runtime;

    // ── GP1 edge detection ────────────────────────────────────────────────
    private boolean prevLBumper1 = false;
    private boolean prevRBumper1 = false;
    private boolean prevA1 = false;
    private boolean prevB1 = false;
    private boolean prevX1 = false;

    // ── GP2 edge detection ────────────────────────────────────────────────
    private boolean prevX2 = false;
    private boolean prevDpadLeft2 = false;
    private boolean prevDpadRight2 = false;
    private boolean prevLTrigger2 = false;
    private boolean prevRTrigger2 = false;
    private boolean prevB2 = false;
    private boolean prevY2 = false;
    private boolean prevA2 = false;

    // ── Carousel tuning state ─────────────────────────────────────────────
    private enum Param { KP, KI, KD, TOLERANCE, MAX_POWER }
    private Param selected = Param.KP;

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

            handleDriverControls();
            handleGunnerControls();
            handleAutoModeSwitching();
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

        if (gamepad1.left_bumper && !prevLBumper1) {
            sensorState.toggleAutoAlign();
            if (sensorState.isAutoAlignEnabled()) {
                driveThread.requestPidReset();
            }
        }
        prevLBumper1 = gamepad1.left_bumper;

        if (gamepad1.right_bumper && !prevRBumper1) {
            if (sensorState.isBasketTagVisible()) sensorState.requestPoseUpdate();
        }
        prevRBumper1 = gamepad1.right_bumper;

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

        double nudgeInput = applyCurve(-gamepad2.left_stick_y);
        if (Math.abs(nudgeInput) > 0.05) {
            int nudgeTicks = (int)(nudgeInput * CarouselController.NUDGE_TICKS);
            mechanismThread.setNudgeRequest(nudgeTicks);
        } else {
            mechanismThread.setNudgeRequest(0);
        }

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
    //  TUNING OVERLAY — GP1 A/B/X for moves, GP2 A + dpad for gains
    // ══════════════════════════════════════════════════════════════════════

    private void handleTuningControls() {
        // GP1 A — rotate +1 slot
        if (gamepad1.a && !prevA1) {
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.ROTATE_RIGHT));
        }
        prevA1 = gamepad1.a;

        // GP1 B — rotate -1 slot
        if (gamepad1.b && !prevB1) {
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.ROTATE_LEFT));
        }
        prevB1 = gamepad1.b;

        // GP1 X — rotate +3 slots (full revolution)
        if (gamepad1.x && !prevX1) {
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.ROTATE_RIGHT));
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.ROTATE_RIGHT));
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.ROTATE_RIGHT));
        }
        prevX1 = gamepad1.x;

        // GP2 A — Cycle selected param
        if (gamepad2.a && !prevA2) {
            Param[] vals = Param.values();
            selected = vals[(selected.ordinal() + 1) % vals.length];
        }
        prevA2 = gamepad2.a;

        // GP2 dpad_up/down — adjust selected param
        double step = getStep();
        if (gamepad2.right_bumper) step *= 10.0;
        if (gamepad2.left_bumper) step *= 0.1;

        if (gamepad2.dpad_up) adjustSelected(step * 0.05);
        if (gamepad2.dpad_down) adjustSelected(-step * 0.05);
    }

    // ══════════════════════════════════════════════════════════════════════
    //  TELEMETRY — matches TeleOp volume, with carousel tuning info
    // ══════════════════════════════════════════════════════════════════════

    private void updateTelemetry() {
        telemetry.addData("Mode", currentMode);
        telemetry.addData("Balls", "%s|%s|%s",
                shortName(sensorState.getPositionColor(0)),
                shortName(sensorState.getPositionColor(1)),
                shortName(sensorState.getPositionColor(2)));

        boolean settled = mechanismThread.isCarouselSettled();
        int cur = mechanismThread.getCarouselCurrentTicks();
        int tgt = mechanismThread.getCarouselTargetTicks();
        telemetry.addData("Carousel", "curr: %d / target: %d (err %+d) | %s",
                cur, tgt, tgt - cur, settled ? "SETTLED" : "MOVING");

        telemetry.addData("State", mechanismThread.getStateDebug());

        telemetry.addData("Tune",
                "%s KP=%.5f KI=%.6f KD=%.6f TOL=%d MAX=%.2f",
                selected.name(),
                CarouselController.KP, CarouselController.KI,
                CarouselController.KD, CarouselController.TOLERANCE,
                CarouselController.MAX_POWER);

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
            case KP:        return 0.0001;
            case KI:        return 0.000005;
            case KD:        return 0.00001;
            case TOLERANCE: return 5;
            case MAX_POWER: return 0.05;
            default:        return 0.0001;
        }
    }

    private void adjustSelected(double delta) {
        switch (selected) {
            case KP:        CarouselController.KP = Math.max(0, CarouselController.KP + delta); break;
            case KI:        CarouselController.KI = Math.max(0, CarouselController.KI + delta); break;
            case KD:        CarouselController.KD = Math.max(0, CarouselController.KD + delta); break;
            case TOLERANCE: CarouselController.TOLERANCE = (int) Math.max(1, CarouselController.TOLERANCE + delta); break;
            case MAX_POWER: CarouselController.MAX_POWER = Math.max(0.05, Math.min(1.0, CarouselController.MAX_POWER + delta)); break;
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
