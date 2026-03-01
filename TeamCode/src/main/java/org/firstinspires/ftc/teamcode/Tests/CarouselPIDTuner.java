package org.firstinspires.ftc.teamcode.threaded.Old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * ══════════════════════════════════════════════════════════════════════════════
 *  CAROUSEL PID TUNER
 * ══════════════════════════════════════════════════════════════════════════════
 *
 *  Use this to find KP, KI, KD for CarouselController without re-deploying.
 *  Once happy, copy the final values into CarouselController.java.
 *
 *  ── GAMEPAD 1 CONTROLS ───────────────────────────────────────────────────────
 *
 *  D-pad Up / Down      — Select which term to adjust (P → I → D → TOLERANCE → MAX_POWER)
 *  D-pad Right / Left   — Increase / decrease selected term by STEP
 *  Right Bumper         — Hold for 10× step (coarse adjust)
 *
 *  A                    — Command ONE slot forward  (+2731 ticks)
 *  B                    — Command ONE slot backward (-2731 ticks)
 *  X                    — Command THREE slots forward (full revolution)
 *  Y                    — Reset encoder + target to zero (re-home)
 *
 *  Left Bumper          — Reset I accumulator mid-move (useful for checking windup)
 *  Start                — Zero out KI and KD  (quick "P-only" baseline)
 *  Back                 — Print current values to telemetry log line (stays visible)
 *
 *  ── TUNING PROCEDURE ─────────────────────────────────────────────────────────
 *
 *  1. Start with  KP = 0.001, KI = 0, KD = 0.
 *  2. Press A. Watch how the carousel moves:
 *       - Too slow / doesn't reach target → raise KP
 *       - Overshoots badly / oscillates   → lower KP
 *  3. Once it reaches target cleanly (maybe small steady-state error), add KD:
 *       - Raise KD until overshoot damps nicely without buzzing.
 *  4. Only add KI if there's a consistent offset at rest. Keep it tiny (< 0.0001).
 *  5. Adjust TOLERANCE if you want a tighter or looser deadband.
 *  6. Copy the logged values into CarouselController.java.
 *
 * ══════════════════════════════════════════════════════════════════════════════
 */
@Disabled
@TeleOp(name = "Carousel PID Tuner", group = "Tuning")
public class CarouselPIDTuner extends LinearOpMode {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private DcMotorEx motor;

    // ── Tunable values (start conservative) ───────────────────────────────────
    private double kP        = 0.002;
    private double kI        = 0.0;
    private double kD        = 0.0;
    private int    tolerance = 50;
    private double maxPower  = 0.98;

    // ── Step sizes ────────────────────────────────────────────────────────────
    private static final double STEP_P         = 0.0001;
    private static final double STEP_I         = 0.000005;
    private static final double STEP_D         = 0.00001;
    private static final int    STEP_TOLERANCE = 5;
    private static final double STEP_MAX_POWER = 0.05;
    private static final double COARSE_MULT    = 10.0;

    // ── Carousel geometry ─────────────────────────────────────────────────────
    private static final int TICKS_PER_SLOT = 2731;
    private static final double INTEGRAL_LIMIT = 0.3;

    // ── PID runtime state ─────────────────────────────────────────────────────
    private int    targetTicks = 0;
    private double integral    = 0;
    private int    lastError   = 0;
    private boolean wasMoving  = false;

    // ── Telemetry helpers ─────────────────────────────────────────────────────
    private enum Param { KP, KI, KD, TOLERANCE, MAX_POWER }
    private Param selected = Param.KP;
    private String savedLine = "";          // last "Back" snapshot

    // ── Edge detection ────────────────────────────────────────────────────────
    private boolean prevDpadUp    = false;
    private boolean prevDpadDown  = false;
    private boolean prevDpadRight = false;
    private boolean prevDpadLeft  = false;
    private boolean prevA         = false;
    private boolean prevB         = false;
    private boolean prevX         = false;
    private boolean prevY         = false;
    private boolean prevStart     = false;
    private boolean prevBack      = false;
    private boolean prevLBumper   = false;

    // ── Loop timing ──────────────────────────────────────────────────────────
    private final ElapsedTime loopTimer = new ElapsedTime();

    // ── Peak tracking ─────────────────────────────────────────────────────────
    private double peakError     = 0;   // largest overshoot/undershoot observed
    private double peakPower     = 0;   // largest power output observed
    private int    moveCount     = 0;   // how many slot moves attempted

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "carousel_motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Carousel PID Tuner ready.");
        telemetry.addLine("Press A/B to move one slot, X for 3 slots, Y to re-home.");
        telemetry.update();
        waitForStart();

        loopTimer.reset();

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();
            dt = Math.max(dt, 0.001);

            handleGamepad(dt);
            double power = runPID(dt);
            updateTelemetry(power);

            // ~10ms loop
            try { Thread.sleep(10); } catch (InterruptedException ignored) {}
        }

        motor.setPower(0);
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  GAMEPAD
    // ══════════════════════════════════════════════════════════════════════════

    private void handleGamepad(double dt) {
        double step = gamepad1.right_bumper ? COARSE_MULT : 1.0;

        // ── Select param ──────────────────────────────────────────────────────
        if (gamepad1.dpad_up && !prevDpadUp) {
            selected = Param.values()[(selected.ordinal() + Param.values().length - 1) % Param.values().length];
        }
        prevDpadUp = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !prevDpadDown) {
            selected = Param.values()[(selected.ordinal() + 1) % Param.values().length];
        }
        prevDpadDown = gamepad1.dpad_down;

        // ── Adjust selected param ─────────────────────────────────────────────
        if (gamepad1.dpad_right && !prevDpadRight) adjust(step, true);
        prevDpadRight = gamepad1.dpad_right;

        if (gamepad1.dpad_left && !prevDpadLeft) adjust(step, false);
        prevDpadLeft = gamepad1.dpad_left;

        // ── Move commands ─────────────────────────────────────────────────────
        if (gamepad1.a && !prevA) {
            move(TICKS_PER_SLOT);
            moveCount++;
            peakError = 0;
            peakPower = 0;
        }
        prevA = gamepad1.a;

        if (gamepad1.b && !prevB) {
            move(-TICKS_PER_SLOT);
            moveCount++;
            peakError = 0;
            peakPower = 0;
        }
        prevB = gamepad1.b;

        if (gamepad1.x && !prevX) {
            move(TICKS_PER_SLOT * 3);
            moveCount++;
            peakError = 0;
            peakPower = 0;
        }
        prevX = gamepad1.x;

        // ── Re-home ───────────────────────────────────────────────────────────
        if (gamepad1.y && !prevY) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            targetTicks = 0;
            integral    = 0;
            lastError   = 0;
            wasMoving   = false;
            peakError   = 0;
            peakPower   = 0;
        }
        prevY = gamepad1.y;

        // ── Reset I accumulator ───────────────────────────────────────────────
        if (gamepad1.left_bumper && !prevLBumper) {
            integral = 0;
        }
        prevLBumper = gamepad1.left_bumper;

        // ── Zero KI/KD (P-only baseline) ─────────────────────────────────────
        if (gamepad1.start && !prevStart) {
            kI = 0;
            kD = 0;
            integral = 0;
        }
        prevStart = gamepad1.start;

        // ── Snapshot to saved line ────────────────────────────────────────────
        if (gamepad1.back && !prevBack) {
            savedLine = String.format("SAVED → KP=%.6f  KI=%.6f  KD=%.6f  TOL=%d  MAX=%.2f",
                    kP, kI, kD, tolerance, maxPower);
        }
        prevBack = gamepad1.back;
    }

    private void adjust(double multiplier, boolean increase) {
        double sign = increase ? 1 : -1;
        switch (selected) {
            case KP:        kP        = Math.max(0, kP        + sign * STEP_P         * multiplier); break;
            case KI:        kI        = Math.max(0, kI        + sign * STEP_I         * multiplier); break;
            case KD:        kD        = Math.max(0, kD        + sign * STEP_D         * multiplier); break;
            case TOLERANCE: tolerance = (int) Math.max(1, tolerance + sign * STEP_TOLERANCE * multiplier); break;
            case MAX_POWER: maxPower  = clamp(maxPower + sign * STEP_MAX_POWER * multiplier, 0.05, 1.0); break;
        }
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  PID  (mirrors CarouselController exactly)
    // ══════════════════════════════════════════════════════════════════════════

    private void move(int deltaTicks) {
        targetTicks += deltaTicks;
        integral   = 0;
        lastError  = targetTicks - motor.getCurrentPosition();
        wasMoving  = true;
    }

    private double runPID(double dt) {
        int curPos = motor.getCurrentPosition();
        int error  = targetTicks - curPos;

        // Track peak overshoot (signed — lets you see if it overshoots direction)
        if (wasMoving && Math.abs(error) > Math.abs(peakError)) {
            peakError = error;
        }

        if (Math.abs(error) <= tolerance) {
            motor.setPower(0);
            wasMoving = false;
            integral  = 0;
            lastError = error;
            return 0;
        }

        integral  += error * dt;
        integral   = clamp(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        double derivative = (error - lastError) / dt;
        lastError  = error;

        double power = kP * error + kI * integral + kD * derivative;
        power = clamp(power, -maxPower, maxPower);
        motor.setPower(power);
        wasMoving = true;

        if (Math.abs(power) > peakPower) peakPower = Math.abs(power);
        return power;
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  TELEMETRY
    // ══════════════════════════════════════════════════════════════════════════

    private void updateTelemetry(double power) {
        int curPos = motor.getCurrentPosition();
        int error  = targetTicks - curPos;

        // ── Status bar ────────────────────────────────────────────────────────
        telemetry.addLine("══ CAROUSEL PID TUNER ══════════════════");

        // ── Position ──────────────────────────────────────────────────────────
        telemetry.addData("Position", "%d  →  target %d  (err %+d)",
                curPos, targetTicks, error);
        telemetry.addData("Status",   wasMoving ? "MOVING" : "SETTLED");

        // ── Power / peaks ─────────────────────────────────────────────────────
        telemetry.addData("Power",    "%+.3f   (peak %.3f)", power, peakPower);
        telemetry.addData("PeakErr",  "%+d ticks  (move #%d)", (int) peakError, moveCount);
        telemetry.addData("Integral", "%.5f", integral);

        telemetry.addLine("─────────────────────────────────────────");

        // ── Tunable params (highlight selected) ───────────────────────────────
        telemetry.addData(tag(Param.KP),        "%.6f", kP);
        telemetry.addData(tag(Param.KI),        "%.6f", kI);
        telemetry.addData(tag(Param.KD),        "%.6f", kD);
        telemetry.addData(tag(Param.TOLERANCE), "%d ticks", tolerance);
        telemetry.addData(tag(Param.MAX_POWER), "%.2f", maxPower);

        telemetry.addLine("─────────────────────────────────────────");

        // ── Controls reminder ─────────────────────────────────────────────────
        telemetry.addLine("DUp/Dn=select  DRt/Lt=adjust  RB=10×");
        telemetry.addLine("A=+slot  B=-slot  X=+3slots  Y=re-home");
        telemetry.addLine("LB=reset I  Start=zero KI+KD  Back=snapshot");

        // ── Snapshot ──────────────────────────────────────────────────────────
        if (!savedLine.isEmpty()) {
            telemetry.addLine("─────────────────────────────────────────");
            telemetry.addLine(savedLine);
        }

        telemetry.update();
    }

    /** Adds a ► marker next to whichever param is currently selected. */
    private String tag(Param p) {
        return (p == selected ? "► " : "  ") + p.name();
    }

    // ══════════════════════════════════════════════════════════════════════════

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}