package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class Robot {

    public DcMotorEx launcherMotor, carouselMotor;

//    public DigitalChannel leftLim, rightLim;

    public ColorSensor blColor, brColor, intakeColor1, intakeColor2;

    public Servo kicker;

    DcMotor Intake1, Intake2;

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
        carouselMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(.1, 0, 0, 0));
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Intake1 = hardwareMap.get(DcMotor.class, "right_intake");
        Intake2 = hardwareMap.get(DcMotor.class, "left_intake");

//        leftLim = hardwareMap.get(DigitalChannel.class, "leftFin");
//        leftLim.setMode(DigitalChannel.Mode.INPUT);
//
//        rightLim = hardwareMap.get(DigitalChannel.class, "rightFin");
//        rightLim.setMode(DigitalChannel.Mode.INPUT);

        blColor = hardwareMap.get(ColorSensor.class, "BL_color");
        brColor = hardwareMap.get(ColorSensor.class, "BR_color");

        kicker = hardwareMap.get(Servo.class, "flicker_servo");
        kicker.setPosition(FLICKER_DOWN);

        light1 = hardwareMap.get(Servo.class, "light1");
        light1.setPosition(0);
        light2 = hardwareMap.get(Servo.class, "light2");
        light2.setPosition(0);
        light3 = hardwareMap.get(Servo.class, "light3");
        light3.setPosition(0);





//        intakeColor1 = hardwareMap.get(ColorSensor.class, "intake_color1");
//        intakeColor2 = hardwareMap.get(ColorSensor.class, "intake_color2");
    }

    public boolean getState(DigitalChannel fin) {
        return !fin.getState();
    }

    public void switchFlick() {
        if (kicker.getPosition() == FLICKER_DOWN) {
            kicker.setPosition(FLICKER_UP);
        } else {
            kicker.setPosition(FLICKER_DOWN);
        }
    }

    public void setLauncherMotor(double speed) {
        launcherMotor.setPower(speed);
    }

    public void setIntakeMotors(double speed) {
        Intake1.setPower(speed);
        Intake2.setPower(speed);
    }
}