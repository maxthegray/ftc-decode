package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Localization.UnifiedLocalization;
import org.firstinspires.ftc.teamcode.Shooter.ShooterCamera;

@TeleOp
public class Autonomous extends LinearOpMode {
    private DriveTrain driveTrain;
    private ShooterCamera shooterCamera;
   public int switchState = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        driveTrain = new DriveTrain(hardwareMap, gamepad1, telemetry);
        shooterCamera = new ShooterCamera(telemetry, hardwareMap);

        while(opModeIsActive() && !isStopRequested()) {

            switch(switchState) {
                default:
                    driveTrain.drive();
                    if (gamepad1.triangle) {
                        switchState = 1;
                    }
                    break;
                case 1:
                    shootPrime();
                    switchState = 2;
                    break;
                case 2:
                    driveTrain.goTo(-4,3,0.2);
                    switchState = 0;
                    break;
            }


        }

    }
    private void shootPrime() {
        while (!shooterCamera.canSeeRedTag() && opModeIsActive() && !isStopRequested()) {
            driveTrain.setDrive(0,0,-.2);
        }
        while (Math.abs(shooterCamera.alignRobotToTagPower()) > 0) {
            driveTrain.setDrive(0,0,-shooterCamera.alignRobotToTagPower());
        }
        for (int i = 0; i < 5; i++) {
            shooterCamera.alignCameraToTag();
            sleep(50);
        }
    }


}
