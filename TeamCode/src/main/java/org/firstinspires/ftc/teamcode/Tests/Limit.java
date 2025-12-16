package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "limit", group = "Testing")
public class Limit extends OpMode {

    private DigitalChannel leftFinLimit;
    private DigitalChannel rightFinLimit;

    @Override
    public void init() {
        leftFinLimit = hardwareMap.get(DigitalChannel.class, "leftFin");
        leftFinLimit.setMode(DigitalChannel.Mode.INPUT);

        rightFinLimit = hardwareMap.get(DigitalChannel.class, "rightFin");
        rightFinLimit.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void loop() {
        telemetry.addData("Left", leftFinLimit.getState());
        telemetry.addData("Right", rightFinLimit.getState());
        telemetry.update();
    }
}