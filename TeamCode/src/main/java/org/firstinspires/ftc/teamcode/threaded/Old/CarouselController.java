package org.firstinspires.ftc.teamcode.threaded.Old;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Controls the carousel motor using a super-Gaussian × sigmoid ramp + P-loop correction.
 *
 * The curve is:  0.1 + 0.88 * e^(-|z|^N) * sigmoid(x)
 * The super-Gaussian controls the ramp up and overall shape.
 * The sigmoid independently controls the ramp down.
 *
 * Tuning guide:
 *   SUPER_GAUSSIAN_N   : controls ramp-up sharpness and flat top (higher = sharper)
 *   NORMALIZED_SIGMA   : width of the super-Gaussian
 *   NORMALIZED_CENTER  : where the super-Gaussian peaks (0–1)
 *   SIGMOID_D          : where the ramp-down centers (higher = later drop-off)
 *   SIGMOID_K          : how sharp the ramp-down is (higher = steeper)
 *
 * Output always stays between MIN_POWER (0.1) and MAX_POWER (0.98).
 */
public class CarouselController {

    private final DcMotorEx motor;
    private int targetTicks = 0;
    private int startTicks = 0;

    // ── Power range ─────────────────────────────────────────────────────
    private static final double MAX_POWER = 0.98;
    private static final double MIN_POWER = 0.10;

    // ── Super-Gaussian constants (controls ramp up + flat top) ──────────
    private static final double NORMALIZED_CENTER  = 0.487;  // Where peak sits (0 = start, 1 = end)
    private static final double NORMALIZED_SIGMA   = 0.45;   // Width of the curve
    private static final double SUPER_GAUSSIAN_N   = 10;     // Flatness exponent (higher = flatter top, sharper ramp up)

    // ── Sigmoid constants (controls ramp down only) ─────────────────────
    private static final double SIGMOID_K          = 10;     // Ramp-down sharpness (higher = steeper)
    private static final double SIGMOID_D          = 0.8;    // Where ramp-down centers (higher = later drop-off)

    // ── Phase-switching thresholds ──────────────────────────────────────
    private static final int THRESHOLD  = 80;   // Switch to P-loop when this close to target
    private static final int TOLERANCE  = 10;    // Considered "settled" within this many ticks

    // ── P-loop correction ───────────────────────────────────────────────
    private static final double KP = 0.01;

    // ── Drift correction ────────────────────────────────────────────────
    private static final double DRIFT_CORRECTION_POWER = 0.07;

    // ── Movement presets ────────────────────────────────────────────────
    public static final int TICKS_PER_SLOT = 8192 / 3;
    public static final int NUDGE_TICKS    = 50;

    private enum Phase {
        GAUSSIAN,   // Ramping with super-Gaussian × sigmoid curve
        P_LOOP,     // Fine-tuning with proportional control
        SETTLED     // At target, applying drift correction if needed
    }

    private Phase currentPhase = Phase.SETTLED;

    public CarouselController(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "carousel_motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Move carousel by delta ticks.
     * Snapshots current position so the curve ramps from wherever we are now.
     */
    public void move(int deltaTicks) {
        startTicks = motor.getCurrentPosition();
        targetTicks += deltaTicks;
        currentPhase = Phase.GAUSSIAN;
    }

    /**
     * Rotate by number of slots (positive = right, negative = left).
     */
    public void rotateSlots(int slots) {
        move(slots * TICKS_PER_SLOT);
    }

    /**
     * Small adjustment.
     */
    public void nudge(int ticks) {
        move(ticks);
    }

    /**
     * Call this every loop to apply the power curve, P-loop, and drift correction.
     * This MUST be called regularly for the carousel to move!
     */
    public void update() {
        int currentPos = motor.getCurrentPosition();
        int error = targetTicks - currentPos;

        switch (currentPhase) {
            case GAUSSIAN:
                if (Math.abs(error) <= THRESHOLD) {
                    currentPhase = Phase.P_LOOP;
                    break;
                }
                motor.setPower(gaussian(currentPos));
                break;

            case P_LOOP:
                if (Math.abs(error) <= TOLERANCE) {
                    currentPhase = Phase.SETTLED;
                    motor.setPower(0);
                    break;
                }
                double pPower = KP * error;
                pPower = Math.max(-1.0, Math.min(1.0, pPower));
                motor.setPower(pPower);
                break;

            case SETTLED:
                if (Math.abs(error) > TOLERANCE) {
                    motor.setPower(DRIFT_CORRECTION_POWER * Math.signum(error));
                } else {
                    motor.setPower(0);
                }
                break;
        }
    }

    /**
     * Direction-aware super-Gaussian × sigmoid ramp.
     *
     * Normalizes position to [0..1], then computes:
     *   power = MIN_POWER + (MAX_POWER - MIN_POWER) * superGaussian(progress) * sigmoid(progress)
     *
     * The super-Gaussian handles the sharp ramp up and flat top.
     * The sigmoid independently pulls the curve down for the ramp down.
     * Output is always between MIN_POWER and MAX_POWER.
     */
    private double gaussian(double x) {
        double totalDistance = targetTicks - startTicks;
        if (Math.abs(totalDistance) < 1) return 0;

        double progress = (x - startTicks) / totalDistance;

        // Super-Gaussian: sharp ramp up + flat top
        double z = Math.abs(progress - NORMALIZED_CENTER) / NORMALIZED_SIGMA;
        double sg = Math.exp(-Math.pow(z, SUPER_GAUSSIAN_N));

        // Sigmoid: controls ramp down independently
        double sigmoid = 1.0 / (1.0 + Math.exp(SIGMOID_K * (progress - SIGMOID_D)));

        double magnitude = MIN_POWER + (MAX_POWER - MIN_POWER) * sg * sigmoid;

        return (totalDistance >= 0) ? magnitude : -magnitude;
    }

    /** Is the motor settled at its target? */
    public boolean isSettled() {
        return currentPhase == Phase.SETTLED
                && Math.abs(targetTicks - motor.getCurrentPosition()) <= TOLERANCE;
    }

    /** Has the main Gaussian ramp finished? P-loop may still be running. */
    public boolean isMainMovementDone() {
        return currentPhase != Phase.GAUSSIAN;
    }

    public int getCurrentTicks()   { return motor.getCurrentPosition(); }
    public int getTargetTicks()    { return targetTicks; }
    public Phase getCurrentPhase() { return currentPhase; }
}