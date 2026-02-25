package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Servo Testi")
public class ServoVoltageTest extends LinearOpMode {

    static final double HOME_POSITION   = 0.0;
    static final double HOME_VOLTAGE    = 1.128;
    static final double FLICK_POSITION  = 0.3;
    static final double FLICK_VOLTAGE   = 1.433;
    static final double THRESHOLD       = 0.05; // volts

    ElapsedTime flickTimer = new ElapsedTime();
    double lastFlickMs = 0;

    @Override
    public void runOpMode() {
        Servo       axon     = hardwareMap.get(Servo.class,       "flicker_servo");
        AnalogInput feedback = hardwareMap.get(AnalogInput.class, "flick");

        boolean prevCircle = false;

        axon.setPosition(HOME_POSITION);
        waitForStart();

        while (opModeIsActive()) {
            double voltage = feedback.getVoltage();

            if (gamepad1.circle && !prevCircle) {
                lastFlickMs = flick(axon, feedback);
            }
            prevCircle = gamepad1.circle;

            telemetry.addData("Voltage",         "%.3f V", voltage);
            telemetry.addData("Last Round Trip",  "%.1f ms", lastFlickMs);
            telemetry.addLine("[Circle] Flick");
            telemetry.update();
        }
    }

    double flick(Servo axon, AnalogInput feedback) {
        flickTimer.reset();

        // Go to flick position, wait for voltage to rise to target
        axon.setPosition(FLICK_POSITION);
        while (opModeIsActive() && feedback.getVoltage() < FLICK_VOLTAGE - THRESHOLD);

        // Return home, wait for voltage to drop back to home
        axon.setPosition(HOME_POSITION);
        while (opModeIsActive() && feedback.getVoltage() > HOME_VOLTAGE + THRESHOLD);

        return flickTimer.milliseconds();
    }
}