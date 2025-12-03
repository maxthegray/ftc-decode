package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Fins", group = "TeleOp")
public class Fins extends LinearOpMode {

    private DigitalChannel leftFinLimit;
    private DigitalChannel rightFinLimit;


    @Override
    public void runOpMode() {

        leftFinLimit = hardwareMap.get(DigitalChannel.class, "leftFin");
        leftFinLimit.setMode(DigitalChannel.Mode.INPUT);

        rightFinLimit = hardwareMap.get(DigitalChannel.class, "rightFin");
        rightFinLimit.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();
        while (opModeIsActive()) {

            boolean leftTriggered = !leftFinLimit.getState();
            boolean rightTriggered = !rightFinLimit.getState();

            telemetry.addData("Left Fin", leftTriggered ? "NP" : "P");
            telemetry.addData("Right Fin", rightTriggered ? "NP" : "P");

            telemetry.update();

        }
    }
}