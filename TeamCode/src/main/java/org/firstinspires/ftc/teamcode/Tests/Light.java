package org.firstinspires.ftc.teamcode.Tests;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Light", group = "Testing")
public class Light extends OpMode {
    Servo light;
    double position;

    @Override
    public void init() {
        light = hardwareMap.get(Servo.class, "light");
        position = 0;
    }

    @Override
    public void loop() {
        light.setPosition(position);
        telemetry.addData("Light Position", light.getPosition());

        light.setPosition(.480);
        sleep(1000);
        light.setPosition(.722);
        sleep(1000);

//        if (gamepad1.dpad_up) {
//            position += .05;
//            sleep(100);
//        }
//        if (gamepad1.dpad_down) {
//            position -=.05;
//            sleep(100);
//        }
        telemetry.update();


    }

}
