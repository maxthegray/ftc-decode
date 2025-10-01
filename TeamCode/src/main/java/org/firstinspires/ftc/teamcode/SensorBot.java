package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Localization.UnifiedLocalization;
import org.firstinspires.ftc.teamcode.Shooter.ShooterCamera;

@TeleOp
public class SensorBot extends LinearOpMode {



    boolean lockedOn = false;

    UnifiedLocalization gps;

    public void runOpMode() throws InterruptedException {

        gps = new UnifiedLocalization(telemetry, hardwareMap);

        DriveTrain driveTrain = new DriveTrain(hardwareMap, gamepad1, telemetry);

        ShooterCamera shooterCamera = new ShooterCamera(telemetry, hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            driveTrain.drive();

            shooterCamera.alignCameraToTag();

            if(gamepad1.dpad_down) {
                driveTrain.goTo(0,0, 0.5);
            }
            if(gamepad1.dpad_up) {
                driveTrain.rampToXYH(0,0,0,1);
            }
            if (gamepad1.square && !lockedOn) {
                driveTrain.lockOntoTag();
                lockedOn = true;
                sleep(100);
            } else if (gamepad1.square && lockedOn) {
                driveTrain.unlockFromTag();
                lockedOn = false;
                sleep(100);
            }


            telemetry.addData("Locked On?", lockedOn);
            shooterCamera.addTelemetry();
            telemetry.update();
        }

    }

}
