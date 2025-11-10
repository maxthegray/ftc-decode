package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.concurrent.TimeUnit;


@TeleOp(name = "detector", group = "Testing")
public class Detector extends OpMode {
    Servo light;
    double position;
    Limelight3A limelight;
    LLResult result = limelight.getLatestResult();
    private Follower follower;
    private double adjustmentCoeficcient;

    int greenPurple = 0; //0 is green 1 is purple

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

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
                adjustmentCoeficcient = result.getTx()/360;
                telemetry.addData("adjustment", adjustmentCoeficcient);
                telemetry.addData("x diff", result.getTx());
            } else {adjustmentCoeficcient = 0;}


    } else if (greenPurple == 1) {
            if (result.isValid()) {
                light.setPosition(.722);
                adjustmentCoeficcient = result.getTx()/360;
                telemetry.addData("adjustment", adjustmentCoeficcient);
                telemetry.addData("x diff", result.getTx());
            } else {adjustmentCoeficcient = 0;}
        }
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x + adjustmentCoeficcient,
                true // Robot Centric
        );

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
