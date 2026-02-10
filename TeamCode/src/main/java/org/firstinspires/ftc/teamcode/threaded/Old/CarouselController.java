package org.firstinspires.ftc.teamcode.threaded.Old;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Controls the carousel motor using Gaussian ramping + P-loop correction.
 */
public class CarouselController {

    private final DcMotorEx motor;
    private int targetTicks = 0;
    private int startTicks = 0;

    // Gaussian constants
    private static final double A = 0.96;           // Peak power
    private static final int THRESHOLD = 80;        // Switch to P-loop when this close to target

    // P-loop correction
    private static final double KP = 0.01;
    private static final int TOLERANCE = 1;

    // Small constant correction for drift
    private static final double DRIFT_CORRECTION_POWER = 0.07;

    public static final int TICKS_PER_SLOT = 780;
    public static final int NUDGE_TICKS = 10;

    private enum Phase {
        GAUSSIAN,   // Ramping with Gaussian curve
        P_LOOP,     // Fine-tuning with P control
        SETTLED     // At target, applying drift correction if needed
    }

    private Phase currentPhase = Phase.SETTLED;

    public CarouselController(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "carousel_motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Changed from RUN_TO_POSITION
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Move carousel by delta ticks.
     * Always snapshots current position so the Gaussian ramps from wherever we are now.
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
     * Call this every loop to apply Gaussian curve, P-loop, and drift correction.
     * This MUST be called regularly for the carousel to move!
     */
    public void update() {
        int currentPos = motor.getCurrentPosition();
        int error = targetTicks - currentPos;

        switch (currentPhase) {
            case GAUSSIAN:
                // Switch to P-loop when close to target
                if (Math.abs(error) <= THRESHOLD) {
                    currentPhase = Phase.P_LOOP;
                    break;
                }

                // Apply Gaussian curve (direction-aware, no sign flip needed)
                double gaussianPower = gaussian(currentPos);
                motor.setPower(gaussianPower);
                break;

            case P_LOOP:
                // Switch to settled when within tolerance
                if (Math.abs(error) <= TOLERANCE) {
                    currentPhase = Phase.SETTLED;
                    motor.setPower(0);
                    break;
                }

                // Apply P-loop correction
                double pPower = KP * error;
                pPower = Math.max(-1.0, Math.min(1.0, pPower));
                motor.setPower(pPower);
                break;

            case SETTLED:
                // Small constant correction for drift
                if (Math.abs(error) > TOLERANCE) {
                    double correctionPower = DRIFT_CORRECTION_POWER * Math.signum(error);
                    motor.setPower(correctionPower);
                } else {
                    motor.setPower(0);
                }
                break;
        }
    }

    /**
     * Direction-aware Gaussian ramp.
     *
     * Normalizes position to a [0..1] fraction of total movement, then applies
     * a Gaussian that peaks near the midpoint.  Works identically for forward
     * and backward moves and scales to any distance.
     *
     * The tuning ratios are preserved from the original constants:
     *   peak  = B / TICKS_PER_SLOT â‰ˆ 0.487  (slightly before midpoint)
     *   width = C / TICKS_PER_SLOT â‰ˆ 0.256
     */
    private double gaussian(double x) {
        double totalDistance = targetTicks - startTicks;
        if (Math.abs(totalDistance) < 1) return 0;

        // How far through the movement are we? (0 = start, 1 = target)
        double progress = (x - startTicks) / totalDistance;

        double normalizedCenter = 0.487;   // was B / TICKS_PER_SLOT
        double normalizedSigma  = 0.256;   // was C / TICKS_PER_SLOT

        double magnitude = A * Math.exp(
                -Math.pow(progress - normalizedCenter, 2)
                        / (2 * normalizedSigma * normalizedSigma));

        // Positive power for forward moves, negative for backward
        return (totalDistance >= 0) ? magnitude : -magnitude;
    }

    /**
     * Is the motor settled at its target?
     */
    public boolean isSettled() {
        return currentPhase == Phase.SETTLED &&
                Math.abs(targetTicks - motor.getCurrentPosition()) <= TOLERANCE;
    }

    public int getCurrentTicks() {
        return motor.getCurrentPosition();
    }

    public int getTargetTicks() {
        return targetTicks;
    }

    public Phase getCurrentPhase() {
        return currentPhase;
    }
}