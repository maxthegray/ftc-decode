package org.firstinspires.ftc.teamcode.threaded;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ShooterThread extends Thread {

    private final BotState state;
    private final DcMotorEx launcherMotor;

    // Tuned PID values
    private static final double SHOOTER_P = 200.0;
    private static final double SHOOTER_I = 0.9;
    private static final double SHOOTER_D = 0.1;
    private static final double SHOOTER_F = 0.0;

    public ShooterThread(BotState state, HardwareMap hardwareMap) {
        this.state = state;
        this.setPriority(Thread.NORM_PRIORITY);

        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher_motor");
        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Apply tuned PID coefficients
        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F));
    }

    @Override
    public void run() {
        while (!state.shouldKillThreads()) {

            double targetVelocity = state.getShooterTargetVelocity();

            // Mutex: only run shooter when intake is not running
            if (state.canShooterRun() && targetVelocity > 0) {
                launcherMotor.setVelocity(targetVelocity, AngleUnit.DEGREES);
            } else {
                launcherMotor.setVelocity(0);
            }

            // Update current velocity in state
            state.setShooterCurrentVelocity(launcherMotor.getVelocity(AngleUnit.DEGREES));

            try {
                Thread.sleep(BotState.SHOOTER_UPDATE_MS);
            } catch (InterruptedException e) {
                break;
            }
        }

        // Cleanup
        launcherMotor.setVelocity(0);
    }
}