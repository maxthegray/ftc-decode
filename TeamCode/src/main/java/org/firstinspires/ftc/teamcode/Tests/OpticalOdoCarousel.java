package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp
public class OpticalOdoCarousel extends LinearOpMode {

    private SparkFunOTOS otos;

    @Override
    public void runOpMode() {
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(offset);

        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);

        otos.calibrateImu();

        otos.resetTracking();


        waitForStart();

        while (opModeIsActive()) {

            SparkFunOTOS.Pose2D position = otos.getPosition();

            SparkFunOTOS.Pose2D velocity = otos.getVelocity();

            SparkFunOTOS.Pose2D acceleration = otos.getAcceleration();

            telemetry.addData("X", position.x);
            telemetry.addData("Y", position.y);
            telemetry.addData("Heading",position.h);

            telemetry.addData("X Velocity",  velocity.x);
            telemetry.addData("Y Velocity", velocity.y);
            telemetry.addData("Angular Velocity",  velocity.h);

            telemetry.addData("X Accel",  acceleration.x);
            telemetry.addData("Y Accel",  acceleration.y);
            telemetry.addData("Angular Accel", acceleration.h);

            if (gamepad1.a) {
                otos.resetTracking();
            }

            telemetry.update();
        }
    }
}