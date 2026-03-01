package org.firstinspires.ftc.teamcode.threaded.Old;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Carousel controller using a PID loop with a hard deadband.
 *
 * Once within TOLERANCE ticks the motor power is cut entirely — no jitter.
 * Only re-engages if the carousel drifts beyond TOLERANCE again.
 *
 * Stall detection: if the encoder moves less than STALL_TICK_THRESHOLD ticks
 * over STALL_TIME_SECONDS while the motor is actively driving, the carousel is
 * assumed to have skipped/jammed.
 * The motor is stopped and the controller
 * force-settles so MechanismThread can react (re-issue the command, alert, etc.).
 *
 * ── TUNING ──────────────────────────────────────────────────────────────────
 *  Start with kP only (kI = kD = 0).
 *
 *  kP  raise from 0.001 until it moves crisply. If it oscillates, lower it.
 *      Good starting point: 0.003
 *  kD  add once kP is set. Helps damp overshoot. Try 0.0001, raise slowly.
 *  kI  add last, only if it consistently falls short of target. Try 0.00005.
 *
 *  MAX_POWER  caps the output so it doesn't slam at full speed. 0.8 is a
 *             good starting point — raise if too slow, lower if too violent.
 */
public class CarouselController {

    private final DcMotorEx motor;

    // ── Tune these ───────────────────────────────────────────────────────────
    private static final double KP        = 0.0012;   // proportional
    private static final double KI        = 0.00000; // integral — set to 0 initially
    private static final double KD        = 0.000001;  // derivative — set to 0 initially
    private static final double MAX_POWER = .98;     // clamp output

    // ── Deadband ─────────────────────────────────────────────────────────────
    private static final int TOLERANCE = 50;  // ticks — power cuts off inside this

    // ── Movement presets ─────────────────────────────────────────────────────
    public static final int TICKS_PER_SLOT = 2731;
    public static final int NUDGE_TICKS    = 50;

    // ── Stall detection ──────────────────────────────────────────────────────
    // If the encoder moves less than STALL_TICK_THRESHOLD ticks within
    // STALL_TIME_SECONDS while the motor is outside the deadband, it is
    // considered stalled and the controller force-settles.
    private static final int    STALL_TICK_THRESHOLD = 2500;    // ticks — min expected movement
    private static final double STALL_TIME_SECONDS   = .5;   // seconds — observation window

    // Stall state — reset on every new move()
    private int    stallCheckPos  = 0;    // encoder position at start of current window
    private double stallTimer     = 0;    // seconds accumulated in current window
    private boolean stalled       = false; // latches true when a stall is detected

    // ── PID state ────────────────────────────────────────────────────────────
    private int    targetTicks   = 0;
    private double integral      = 0;
    private int    lastError     = 0;
    private boolean wasMoving    = false; // true until we first enter the deadband

    private static final double INTEGRAL_LIMIT = 0.3; // anti-windup

    public CarouselController(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "carousel_motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void move(int deltaTicks) {
        targetTicks  += deltaTicks;
        integral      = 0;
        lastError     = targetTicks - motor.getCurrentPosition();
        wasMoving     = true;

        // Reset stall detection for the new movement
        stallCheckPos = motor.getCurrentPosition();
        stallTimer    = 0;
        stalled       = false;
    }

    public void rotateSlots(int slots) { move(slots * TICKS_PER_SLOT); }
    public void nudge(int ticks)       { move(ticks); }

    /**
     * Call every loop with actual elapsed time in seconds.
     * MechanismThread already has a loopTimer — pass dt from there.
     */
    public void update(double dt) {
        dt = Math.max(dt, 0.001); // guard against zero

        int curPos = motor.getCurrentPosition();
        int error  = targetTicks - curPos;

        // Inside deadband — cut power and stay settled
        if (Math.abs(error) <= TOLERANCE) {
            motor.setPower(0);
            wasMoving  = false;
            integral   = 0;
            lastError  = error;
            stallTimer = 0;             // reset stall window; we're done moving
            stallCheckPos = curPos;
            stalled    = false;
            return;
        }

        // ── Stall detection ──────────────────────────────────────────────────
        // Only check while the motor is actively being commanded (wasMoving).
        // Accumulate time; if the encoder hasn't moved enough within the window,
        // force-settle and latch stalled=true so callers can react.
        stallTimer += dt;

        if (stallTimer >= STALL_TIME_SECONDS) {
            int ticksMoved = Math.abs(curPos - stallCheckPos);

            if (ticksMoved < STALL_TICK_THRESHOLD) {
                // Stall confirmed — stop and force-settle
                motor.setPower(0);
                wasMoving = false;
                integral  = 0;
                stalled   = true;
                // Snap targetTicks to current position so isSettled() returns true
                targetTicks = curPos;
                return;
            }

            // Enough movement — reset the window for the next check
            stallCheckPos = curPos;
            stallTimer    = 0;
        }

        // ── PID ──────────────────────────────────────────────────────────────
        integral  += error * dt;
        integral   = clamp(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        double derivative = (error - lastError) / dt;
        lastError  = error;

        double power = KP * error + KI * integral + KD * derivative;
        power = clamp(power, -MAX_POWER, MAX_POWER);
        motor.setPower(power);
        wasMoving = true;
    }

    /** Backwards-compatible no-arg version — uses a fixed 10ms dt (the MechanismThread sleep). */
    public void update() { update(0.01); }

    public boolean isSettled() {
        return !wasMoving
                && Math.abs(targetTicks - motor.getCurrentPosition()) <= TOLERANCE;
    }

    public boolean isMainMovementDone() { return isSettled(); }

    /**
     * True if the last movement ended due to a detected stall rather than
     * reaching the target. Cleared automatically on the next move() call.
     * MechanismThread can poll this to decide whether to retry or alert.
     */
    public boolean isStalled() { return stalled; }

    public int getCurrentTicks() { return motor.getCurrentPosition(); }
    public int getTargetTicks()  { return targetTicks; }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}