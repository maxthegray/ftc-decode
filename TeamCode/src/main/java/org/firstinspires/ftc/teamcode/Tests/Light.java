package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "Light", group = "TeleOp")
public class Light extends OpMode {
    private Servo light1; //right
    private Servo light2; //middle
    private Servo light3; //left

    private double value1 = .7;
    private double value2 = .3;
    private double value3 = 1;

    @Override
    public void init() {
        light1 = hardwareMap.get(Servo.class, "light1");
        light2 = hardwareMap.get(Servo.class, "light2");
        light3 = hardwareMap.get(Servo.class, "light3");
    }

    @Override
    public void loop() {
        handleInputs();
        update();
        light1.setPosition(value1);
        light2.setPosition(value2);
        light3.setPosition(value3);
        telemetry.addData("light1", light1.getPosition());
        telemetry.addData("light2", light2.getPosition());
        telemetry.addData("light3", light3.getPosition());
        telemetry.update();
    }

    public void update() {

    }
    public void handleInputs() {
        if (gamepad1.dpad_right) {
            value1 += 0.05;
        } else if (gamepad1.dpad_left) {
            value1 -= 0.05;
        }

        if (gamepad1.dpad_up) {
            value2 += 0.05;
        } else if (gamepad1.dpad_down) {
            value2 -= 0.05;
        }

        if (gamepad1.right_bumper) {
            value3 += 0.05;
        } else if (gamepad1.left_bumper){
            value3 -= 0.05;
        }

    }
}
