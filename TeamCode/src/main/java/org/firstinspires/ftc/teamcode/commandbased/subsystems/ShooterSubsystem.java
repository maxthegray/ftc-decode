package org.firstinspires.ftc.teamcode.commandbased.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class ShooterSubsystem extends SubsystemBase {

    private final DcMotor shooterMotor;
    private double targetPower = 0;

    private static final double POWER_INCREMENT = 0.05;
    private static final double MIN_POWER = 0.0;
    private static final double MAX_POWER = 1.0;
    private static final double POWER_TOLERANCE = 0.02;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotor.class, "launcherMotor");
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void periodic() {
        // Update motor power
        shooterMotor.setPower(targetPower);
    }

    public void setTargetPower(double power) {
        targetPower = Range.clip(power, MIN_POWER, MAX_POWER);
    }

    public void increasePower() {
        targetPower = Range.clip(targetPower + POWER_INCREMENT, MIN_POWER, MAX_POWER);
    }

    public void decreasePower() {
        targetPower = Range.clip(targetPower - POWER_INCREMENT, MIN_POWER, MAX_POWER);
    }

    public void stop() {
        targetPower = 0;
        shooterMotor.setPower(0);
    }

    public double getTargetPower() {
        return targetPower;
    }

    public double getCurrentPower() {
        return shooterMotor.getPower();
    }

    public boolean isAtTargetSpeed() {
        // Check if motor is at target power (within tolerance)
        return targetPower > 0 &&
                Math.abs(shooterMotor.getPower() - targetPower) <= POWER_TOLERANCE;
    }
}