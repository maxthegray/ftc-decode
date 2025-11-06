package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "Light", group = "Testing")
public class Light extends OpMode {
    Servo light;

    public void init() {
        light = hardwareMap.get(Servo.class, "light");
    }

    public void loop() {
        telemetry.addData("Light Position", light.getPosition());

        if (gamepad1.cross) {
            light.setPosition(1.0); // Turn on the light
        }
        if (gamepad1.square) {
            light.setPosition(0);
        }
        telemetry.update();

    }

}
