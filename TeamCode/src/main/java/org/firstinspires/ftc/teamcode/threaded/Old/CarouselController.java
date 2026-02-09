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

    // Gaussian constants (from SpinTest)
    private static final double A = 0.96;
    private static final double B = 380.0;
    private static final double C = 200.0;
    private static final int THRESHOLD = 760;

    // P-loop correction
    private static final double KP = 0.01;
    private static final int TOLERANCE = 5;

    // Small constant correction for drift
    private static final double DRIFT_CORRECTION_POWER = 0.05;

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
     */
    public void move(int deltaTicks) {
        if (isSettled()) {
            startTicks = motor.getCurrentPosition();
        }
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

                // Apply Gaussian curve
                double gaussianPower = gaussian(currentPos);

                // Reverse direction if moving backward
                if (targetTicks < startTicks) {
                    gaussianPower = -gaussianPower;
                }

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
     * Gaussian curve centered at startTicks + B.
     * This creates a smooth ramp that peaks partway through the movement.
     */
    private double gaussian(double x) {
        double center = startTicks + B;
        return A * Math.exp(-Math.pow(x - center, 2) / (2 * Math.pow(C, 2)));
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