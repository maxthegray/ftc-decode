package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Fins", group = "TeleOp")
public class Fins extends LinearOpMode {

    private DigitalChannel leftFinLimit;
    private DigitalChannel rightFinLimit;
    private boolean clockwise = true;
    private boolean betweenFins = false;



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
    private void IsAtPos() {
        boolean leftTriggered = !leftFinLimit.getState();
        boolean rightTriggered = !rightFinLimit.getState();
        if (clockwise) {
            if (!rightTriggered) {
                betweenFins = false;
            } else if (leftTriggered) {
                betweenFins = true;
            }
        } else {
            if (leftTriggered) {
                betweenFins = false;
            } else if (rightTriggered) {
                betweenFins = true;
            }
        }
    }
}