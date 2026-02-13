package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;


@Disabled
@TeleOp
public class ServoVoltageTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo axon = hardwareMap.get(Servo.class, "flicker_servo");
        AnalogInput feedback = hardwareMap.get(AnalogInput.class, "flick");

        double position = 0.5;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                position += 0.005;
            } else if (gamepad1.dpad_down) {
                position -= 0.005;
            }

            position = Math.max(0, Math.min(1, position));

            axon.setPosition(position);

            double voltage = feedback.getVoltage();
            double feedbackPosition = voltage / 3.3;

            telemetry.addData("Target", "%.3f", position);
            telemetry.addData("Feedback Voltage", "%.3f V", voltage);
            telemetry.addData("Feedback Position", "%.3f", feedbackPosition);
            telemetry.update();
        }
    }
}