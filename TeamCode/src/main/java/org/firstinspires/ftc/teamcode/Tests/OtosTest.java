package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp(name = "OTOS Test", group = "Test")
public class OtosTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        SparkFunOTOS otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.calibrateImu();
        otos.resetTracking();

        waitForStart();

        while (opModeIsActive()) {
            SparkFunOTOS.Pose2D pose = otos.getPosition();

            telemetry.addLine("Push robot manually and check values:");
            telemetry.addLine("");
            telemetry.addData("X", "%.2f in (should increase pushing FORWARD)", pose.x);
            telemetry.addData("Y", "%.2f in (should increase pushing LEFT)", pose.y);
            telemetry.addData("H", "%.2f° (should increase turning LEFT/CCW)", pose.h);
            telemetry.update();
        }
    }
}