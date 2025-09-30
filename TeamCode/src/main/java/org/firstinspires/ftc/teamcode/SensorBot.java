package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class SensorBot extends LinearOpMode {



    boolean lockedOn = false;

    UnifiedLocalization gps;

    public void runOpMode() throws InterruptedException {

        gps = new UnifiedLocalization(telemetry, hardwareMap, 0,0,0);

        DriveTrain driveTrain = new DriveTrain(hardwareMap, gamepad1, gps);

        waitForStart();

        gps.configureOtos();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            driveTrain.drive();

            if(gamepad1.dpad_down) {
                driveTrain.goTo(0,0, 0.05);
            }
            if(gamepad1.dpad_up) {
                driveTrain.rampToXY(0,0,0.1);
            }


            if (gamepad1.square && !lockedOn) {
                driveTrain.lockOntoTag(24);
                lockedOn = true;
            } else if (gamepad1.square && lockedOn) {
                driveTrain.unlockFromTag();
                lockedOn = false;
            }



            gps.step();
            gps.addTelemetry();
            telemetry.update();
        }

    }

}
