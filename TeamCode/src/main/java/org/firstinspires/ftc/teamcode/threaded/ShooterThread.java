package org.firstinspires.ftc.teamcode.threaded;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ShooterThread extends Thread {

    private final BotState state;
    private final DcMotorEx launcherMotor;

    private static final long UPDATE_INTERVAL_MS = 1000;

    public ShooterThread(BotState state, HardwareMap hardwareMap) {
        this.state = state;

        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher_motor");
        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
                Thread.sleep(UPDATE_INTERVAL_MS);
            } catch (InterruptedException e) {
                break;
            }
        }

        // Cleanup
        launcherMotor.setVelocity(0);
    }
}