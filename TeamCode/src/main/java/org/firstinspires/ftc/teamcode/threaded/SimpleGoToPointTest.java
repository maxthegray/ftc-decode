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

        // Go to the right
        goTo.goToPoint(24, -24, 0);
        while (opModeIsActive() && !goTo.isDone()) {
            goTo.update();
            telemetry.addData("Going to", "(24, -24, 0)");
            telemetry.addData("Position", "(%.1f, %.1f) %.1f°", goTo.getX(), goTo.getY(), goTo.getHeading());
            telemetry.update();
        }

        sleep(500);

        // Return to start and turn 90 degrees
        goTo.goToPoint(0, 0, 90);
        while (opModeIsActive() && !goTo.isDone()) {
            goTo.update();
            telemetry.addData("Going to", "(0, 0, 90)");
            telemetry.addData("Position", "(%.1f, %.1f) %.1f°", goTo.getX(), goTo.getY(), goTo.getHeading());
            telemetry.update();
        }

        telemetry.addData("Status", "Done!");
        telemetry.update();
        sleep(2000);
    }
}