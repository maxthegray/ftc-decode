package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevTouchSensor;
@Disabled
@TeleOp(name = "Limit Switch Test", group = "Test")
public class LimitTest extends LinearOpMode {

    private RevTouchSensor limitSwitch;

    @Override
    public void runOpMode() {
        limitSwitch = hardwareMap.get(RevTouchSensor.class, "limitSwitch");

        waitForStart();

        while (opModeIsActive()) {
            boolean isPressed = limitSwitch.isPressed();

            telemetry.addData("Is Pressed", isPressed);
            telemetry.addData("Value", limitSwitch.getValue());
            telemetry.addLine();
            telemetry.addData("Status", isPressed ? ">>> TRIGGERED <<<" : "Not triggered");
            telemetry.update();
        }
    }
}