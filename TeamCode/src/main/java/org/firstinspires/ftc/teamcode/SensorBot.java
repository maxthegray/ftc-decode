package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class SensorBot extends LinearOpMode {



    UnifiedLocalization gps;

    public void runOpMode() throws InterruptedException {

        gps = new UnifiedLocalization(telemetry, hardwareMap, 0,0,0);

        DriveTrain driveTrain = new DriveTrain(hardwareMap, gamepad1, gps);

        waitForStart();

        gps.configureOtos();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            driveTrain.drive();

            if(gamepad1.circle) {
                driveTrain.driveToY(-30, 0.05);

            }

            gps.step();
            gps.addTelemetry();
            telemetry.update();
        }

    }

}
