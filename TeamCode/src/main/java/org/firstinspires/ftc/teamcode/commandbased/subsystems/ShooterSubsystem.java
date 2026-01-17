package org.firstinspires.ftc.teamcode.commandbased.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx launcherMotor;
    private double targetVelocity = 0; // degrees per second

    // === OPTIMIZATION: Track last written velocity to avoid redundant I2C writes ===
    private double lastWrittenVelocity = -1; // Initialize to impossible value to force first write

    public ShooterSubsystem(HardwareMap hardwareMap) {
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher_motor");

        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(4, 2, 0, 5));
    }

    @Override
    public void periodic() {
        // === OPTIMIZATION: Only write velocity when it changes ===
        if (targetVelocity != lastWrittenVelocity) {
            launcherMotor.setVelocity(targetVelocity, AngleUnit.DEGREES);
            lastWrittenVelocity = targetVelocity;
        }
    }

    public void setTargetVelocity(double degreesPerSecond) {
        targetVelocity = degreesPerSecond;
    }

    public void setAdjustedVelocity(double d) {
        double a = 1;
        double b = 1;
        double c = 1;
        targetVelocity = a*d*d + b*d + c; // TODO: Calibrate this equation
    }

    public void increaseVelocity(double delta) {
        targetVelocity = Math.max(0, targetVelocity + delta);
    }

    public void decreaseVelocity(double delta) {
        targetVelocity = Math.max(0, targetVelocity - delta);
    }

    public void stop() {
        targetVelocity = 0;
        // Force immediate write for stop command
        launcherMotor.setPower(0);
        lastWrittenVelocity = 0;
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
}