package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name = "Servo Testi")
public class ServoVoltageTest extends LinearOpMode {

    static final double HOME_POSITION   = 0.0;
    static final double HOME_VOLTAGE    = 1.128;
    static final double FLICK_POSITION  = 0.4217;
    static final double FLICK_VOLTAGE   = 1.568;
    static final double THRESHOLD       = 0.05; // volts
    static final double ADJUST_SPEED    = 0.05; // position units per loop tick

    ElapsedTime flickTimer = new ElapsedTime();
    double lastFlickMs = 0;

    @Override
    public void runOpMode() {
        Servo       axon     = hardwareMap.get(Servo.class,       "flicker_servo");
        AnalogInput feedback = hardwareMap.get(AnalogInput.class, "flick");

        boolean prevCircle = false;
        double manualPosition = HOME_POSITION;

        axon.setPosition(HOME_POSITION);
        waitForStart();

        while (opModeIsActive()) {
            double voltage = feedback.getVoltage();

            if (gamepad1.circle && !prevCircle) {
                lastFlickMs = flick(axon, feedback);
                manualPosition = HOME_POSITION; // sync after flick
            } else {
                // Left stick Y adjusts position (up = increase, negate because stick up = negative)
                double stick = -gamepad1.left_stick_y;
                if (Math.abs(stick) > 0.05) {
                    manualPosition += stick * ADJUST_SPEED;
                    manualPosition = Math.max(0.0, Math.min(1.0, manualPosition));
                    axon.setPosition(manualPosition);
                }
            }
            prevCircle = gamepad1.circle;

            telemetry.addData("Voltage",          "%.3f V", voltage);
            telemetry.addData("Servo Position",   "%.4f",   manualPosition);
            telemetry.addData("Last Round Trip",  "%.1f ms", lastFlickMs);
            telemetry.addLine("---");
            telemetry.addLine("[Circle]        Flick");
            telemetry.addLine("[Left Stick Y]  Adjust position");
            telemetry.update();
        }
    }

    double flick(Servo axon, AnalogInput feedback) {
        flickTimer.reset();

        axon.setPosition(FLICK_POSITION);
        while (opModeIsActive() && feedback.getVoltage() < FLICK_VOLTAGE - THRESHOLD) {
            telemetry.addData("Phase",   "Flicking...");
            telemetry.addData("Voltage", "%.3f V", feedback.getVoltage());
            telemetry.update();
        }

        axon.setPosition(HOME_POSITION);
        while (opModeIsActive() && feedback.getVoltage() > HOME_VOLTAGE + THRESHOLD) {
            telemetry.addData("Phase",   "Returning...");
            telemetry.addData("Voltage", "%.3f V", feedback.getVoltage());
            telemetry.update();
        }

        return flickTimer.milliseconds();
    }
}