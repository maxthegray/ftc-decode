package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import java.util.concurrent.TimeUnit;
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


        if (gamepad1.dpad_up) {
            position +=.05;
            sleep(100);
        }
        if (gamepad1.dpad_down) {
            position -=.05;
            sleep(100);
        }


    }

}
