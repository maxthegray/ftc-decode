package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Localization.UnifiedLocalization;
import org.firstinspires.ftc.teamcode.Shooter.ShooterCamera;

@TeleOp
public class SensorBot extends LinearOpMode {



    boolean lockedOn = false;


    public void runOpMode() throws InterruptedException {

        UnifiedLocalization Location = new UnifiedLocalization(telemetry, hardwareMap);

        DriveTrain driveTrain = new DriveTrain(hardwareMap, gamepad1, telemetry);

        ShooterCamera shooterCamera = new ShooterCamera(telemetry, hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            driveTrain.drive();

            if(gamepad1.dpad_down) {
                driveTrain.goTo(0,0, 0.5);
            }
            if(gamepad1.dpad_up) {
                driveTrain.rampToXYH(0,0,0,1);
            }

            if (lockedOn) {
                driveTrain.unlockFromTag();
                lockedOn = false;
                sleep(50);
            } else {
                driveTrain.lockOntoTag();
                lockedOn = true;
                sleep(50);
            }



            telemetry.update();
        }

    }

}
