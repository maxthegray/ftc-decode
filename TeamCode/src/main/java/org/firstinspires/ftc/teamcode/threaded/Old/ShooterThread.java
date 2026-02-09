package org.firstinspires.ftc.teamcode.threaded.Old;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Controls the shooter motor.
 */
public class ShooterThread extends Thread {

    private final SensorState state;
    private final DcMotorEx motor;

    private static final double SHOOTER_P = 200.0;
    private static final double SHOOTER_I = 0.9;
    private static final double SHOOTER_D = 0.1;

    public ShooterThread(SensorState state, HardwareMap hardwareMap) {
        this.state = state;
        this.setPriority(Thread.NORM_PRIORITY);

        motor = hardwareMap.get(DcMotorEx.class, "launcher_motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, 0));
    }

    @Override
    public void run() {
        while (!state.shouldKillThreads()) {
            double target = state.getShooterTargetVelocity();

            if (target > 0) {
                motor.setVelocity(target, AngleUnit.DEGREES);
            } else {
                motor.setVelocity(0);
            }

            state.setShooterCurrentVelocity(motor.getVelocity(AngleUnit.DEGREES));

            try {
                Thread.sleep( SensorState.SHOOTER_UPDATE_MS);
            } catch (InterruptedException e) {
                break;
            }
        }

        motor.setVelocity(0);
    }
}