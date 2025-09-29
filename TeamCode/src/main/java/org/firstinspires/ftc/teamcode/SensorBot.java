package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class SensorBot extends LinearOpMode {



    UnifiedLocalization gps;

    double X;
    double Y;
    double H;



    public void runOpMode() throws InterruptedException {

        gps = new UnifiedLocalization(telemetry, hardwareMap, 0,0,0);

        DriveTrain driveTrain = new DriveTrain(hardwareMap, gamepad1);

        waitForStart();

        gps.configureOtos();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            driveTrain.drive();

            gps.step();
            gps.addTelemetry();
            telemetry.update();
        }

    }

}
