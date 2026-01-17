package org.firstinspires.ftc.teamcode.commandbased.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx launcherMotor;
    private double targetVelocity = 0; // degrees per second

    // Configurable update rate (milliseconds between updates)
    public static long UPDATE_INTERVAL_MS = 100;
    private final ElapsedTime updateTimer = new ElapsedTime();

    // Distance-to-velocity coefficients (quadratic: a*d^2 + b*d + c)
    private static final double COEFF_A = -0.00983492;
    private static final double COEFF_B = 2.07959;
    private static final double COEFF_C = 93.57352;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher_motor");
        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void periodic() {
        // Throttle updates based on configurable interval
        if (updateTimer.milliseconds() < UPDATE_INTERVAL_MS) {
            return;
        }
        updateTimer.reset();

        launcherMotor.setVelocity(targetVelocity, AngleUnit.DEGREES);
    }

    /**
     * Set shooter velocity based on distance to target using quadratic formula.
     * velocity = a*d^2 + b*d + c
     * @param distance Distance to tag in inches
     */
    public void setAdjustedVelocity(double distance) {
        targetVelocity = COEFF_A * distance * distance + COEFF_B * distance + COEFF_C;
        // Clamp to reasonable range
        targetVelocity = Math.max(0, targetVelocity);
    }

    public void setTargetVelocity(double degreesPerSecond) {
        targetVelocity = degreesPerSecond;
    }

    public void stop() {
        targetVelocity = 0;
        launcherMotor.setVelocity(0);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getCurrentVelocity() {
        return launcherMotor.getVelocity(AngleUnit.DEGREES);
    }

    public boolean isAtTargetVelocity(double tolerance) {
        return targetVelocity > 0 &&
                Math.abs(getCurrentVelocity() - targetVelocity) <= tolerance;
    }

    // Configurable update rate
    public static void setUpdateInterval(long intervalMs) {
        UPDATE_INTERVAL_MS = intervalMs;
    }

    public static long getUpdateInterval() {
        return UPDATE_INTERVAL_MS;
    }
}