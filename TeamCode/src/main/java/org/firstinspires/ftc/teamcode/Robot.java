package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class Robot {

    public DcMotorEx launcherMotor, carouselMotor;

    public DigitalChannel leftLim, rightLim;

    public ColorSensor blColor, brColor, intakeColor1, intakeColor2;

    public Servo kicker;

    //Lights

    public Servo light1, light2, light3;

    public static final double FLICKER_DOWN = 0.0;
    public static final double FLICKER_UP = 0.4;

    private HardwareMap hardwareMap;

    public Robot(HardwareMap hwMap) {
        this.hardwareMap = hwMap;
    }

    public void init() {
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher_motor");
        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        carouselMotor = hardwareMap.get(DcMotorEx.class, "carousel_motor");
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLim = hardwareMap.get(DigitalChannel.class, "leftFin");
        leftLim.setMode(DigitalChannel.Mode.INPUT);

        rightLim = hardwareMap.get(DigitalChannel.class, "rightFin");
        rightLim.setMode(DigitalChannel.Mode.INPUT);

        blColor = hardwareMap.get(ColorSensor.class, "BL_color");
        brColor = hardwareMap.get(ColorSensor.class, "BR_color");

        kicker = hardwareMap.get(Servo.class, "flicker_servo");
        kicker.setPosition(FLICKER_DOWN);

//        light1 = hardwareMap.get(Servo.class, "light1");
//        light1.setPosition(0);
//        light2 = hardwareMap.get(Servo.class, "light2");
//        light2.setPosition(0);
//        light3 = hardwareMap.get(Servo.class, "light3");
//        light3.setPosition(0);


        intakeColor1 = hardwareMap.get(ColorSensor.class, "intake_color1");
        intakeColor2 = hardwareMap.get(ColorSensor.class, "intake_color2");
    }

    public boolean getState(DigitalChannel fin) {
        return !fin.getState();
    }
}