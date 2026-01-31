package org.firstinspires.ftc.teamcode.threaded.Old;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.threaded.Old.SimpleDriveToPoint;

import java.util.AbstractMap;

@Autonomous(name = "GoToPoint Test", group = "Test")
public class OtosDriveExample extends LinearOpMode {

    @Override
    public void runOpMode() {
        SimpleDriveToPoint driver = new SimpleDriveToPoint(hardwareMap);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        // Drive forward 24 inches
        while (opModeIsActive() && !driver.update(24, 0, 0)) {
            SparkFunOTOS.Pose2D pose = driver.getPosition();
            telemetry.addData("Target", "24, 0, 0");
            telemetry.addData("X", "%.1f", pose.x);
            telemetry.addData("Y", "%.1f", pose.y);
            telemetry.addData("H", "%.1f", pose.h);
            telemetry.update();
        }

        sleep(1000);

        // Drive back
        while (opModeIsActive() && !driver.update(0, 0, 0)) {
            SparkFunOTOS.Pose2D pose = driver.getPosition();
            telemetry.addData("Target", "0, 0, 0");
            telemetry.addData("X", "%.1f", pose.x);
            telemetry.addData("Y", "%.1f", pose.y);
            telemetry.addData("H", "%.1f", pose.h);
            telemetry.update();
        }

        telemetry.addData("Status", "Done!");
        telemetry.update();
        sleep(2000);
    }
}