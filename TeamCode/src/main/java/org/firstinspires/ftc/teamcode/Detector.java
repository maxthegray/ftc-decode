package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "detector", group = "Testing")
@Disabled
public class Detector extends OpMode {
    Servo light;
    double position;
    Limelight3A limelight;
    LLResult result = limelight.getLatestResult();
    private Follower follower;
    private double adjustment = 0;

    int greenPurple = 0; //0 is green 1 is purple

    DcMotor frontLeftMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backRightMotor = null;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        light = hardwareMap.get(Servo.class, "light");
        position = 0;
        limelight = hardwareMap.get(Limelight3A.class, "slur");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
    }

    @Override
    public void loop() {
        if (greenPurple == 0) {
            if (result.isValid()) {
                light.setPosition(.475);
                adjustment = result.getTx()/360;
                telemetry.addData("adjustment", adjustment);
                telemetry.addData("x diff", result.getTx());
            } else {adjustment = 0;}

        } else if (greenPurple == 1) {
            if (result.isValid()) {
                light.setPosition(.722);
                adjustment = result.getTx()/360;
                telemetry.addData("adjustment", adjustment);
                telemetry.addData("x diff", result.getTx());
            } else {
                adjustment = 0;}
        }
        frontLeftMotor.setPower(-adjustment);
        backLeftMotor.setPower(-adjustment);
        frontRightMotor.setPower(adjustment);
        backRightMotor.setPower(adjustment);


        telemetry.update();

        if (gamepad1.dpad_up) {
            if (greenPurple == 1) {
                limelight.pipelineSwitch(0);
                greenPurple = 0;
                sleep(200);
            }
            if (greenPurple == 0) {
                limelight.pipelineSwitch(1);
                greenPurple = 1;
                sleep(200);
            }
        }

    }

}
