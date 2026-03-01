package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.Localization.UnifiedLocalization;
import org.firstinspires.ftc.teamcode.Shooter.ShooterCamera;

@Disabled
@TeleOp
public class SensorBot extends LinearOpMode {

    private boolean lockedOn = false;
    private DriveTrain driveTrain;
    private ShooterCamera shooterCamera;
    private int state = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        driveTrain = new DriveTrain(hardwareMap, gamepad1, telemetry);
        //shooterCamera = new ShooterCamera(telemetry, hardwareMap);

        waitForStart();


        if (isStopRequested()) {
            return;
        }
        boolean launcherOn = false;
        double motorPower = 0.5;

        while (opModeIsActive() && !isStopRequested()) {

            if (launcherOn==true) {
                driveTrain.launcherMotor.setPower(motorPower);
            } else {
                driveTrain.launcherMotor.setPower(0);
            }

            if (gamepad1.dpad_up) {
                motorPower += 0.05;
            }
            if (gamepad1.dpad_down){
                motorPower -= 0.05;
            }
            driveTrain.drive();

            if (gamepad1.square) {
               // shootPrime();
               launcherOn = !launcherOn;
            }
            telemetry.addData("Up on Dpad to increase power, down on Dpad to decrease power", "");
            telemetry.addData("Power", motorPower);
            telemetry.addData("Press square to turn on/off motor", "");
            telemetry.update();


        }

    }

//    private void shootPrime() {
//        while (!shooterCamera.canSeeRedTag() && opModeIsActive() && !isStopRequested()) {
//            driveTrain.setDrive(0,0,-.2);
//        }
//        while (Math.abs(shooterCamera.alignRobotToTagPower()) > 0) {
//            driveTrain.setDrive(0,0,-shooterCamera.alignRobotToTagPower());
//        }
//        for (int i = 0; i < 5; i++) {
//            shooterCamera.alignCameraToTag();
//            sleep(50);
//        }
//    }





}
