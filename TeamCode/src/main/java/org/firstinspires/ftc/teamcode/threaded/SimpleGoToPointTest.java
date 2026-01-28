package org.firstinspires.ftc.teamcode.threaded;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Simple Go To Point Test")
public class SimpleGoToPointTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        SimpleGoToPoint goTo = new SimpleGoToPoint(hardwareMap);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        // Go forward 24 inches
        goTo.goToPoint(24, 0, 0);
        while (opModeIsActive() && !goTo.isDone()) {
            goTo.update();
            telemetry.addData("Going to", "(24, 0, 0)");
            telemetry.addData("Position", "(%.1f, %.1f) %.1f°", goTo.getX(), goTo.getY(), goTo.getHeading());
            telemetry.update();
        }

        sleep(500);

        telemetry.addData("Status", "Done!");
        telemetry.update();
        sleep(2000);
    }
}