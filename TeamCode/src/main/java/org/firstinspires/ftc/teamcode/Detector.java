package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import java.util.concurrent.TimeUnit;
@TeleOp(name = "detector", group = "Testing")
public class Detector extends OpMode {
    Servo light;
    double position;
    Limelight3A limelight;
    LLResult result = limelight.getLatestResult();

    int greenPurple = 0; //0 is green 1 is purple

    @Override
    public void init() {
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
                telemetry.addData("x diff", result.getTx());
            }
        } else if (greenPurple == 1) {
            if (result.isValid()) {
                light.setPosition(.722);
                telemetry.addData("x diff", result.getTx());
            }
        }
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
