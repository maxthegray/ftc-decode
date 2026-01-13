package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Light", group = "TeleOp")
public class Light extends OpMode {
    private Servo rightLights;
    private Servo middleLights;
    private Servo leftLights;

    private double rightValue;
    private double leftValue;
    private double middleValue;

    @Override
    public void init() {
        rightLights = hardwareMap.get(Servo.class, "right_lights");
        middleLights = hardwareMap.get(Servo.class, "middle_lights");
        leftLights = hardwareMap.get(Servo.class, "left_lights");
        middleValue = .600;
        rightValue = .900;
        leftValue = .750;
    }

    @Override
    public void loop() {
        update();
    }

    public void update() {
        rightLights.setPosition(rightValue);
        middleLights.setPosition(middleValue);
        leftLights.setPosition(leftValue);

    }
}
