package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.Localization.UnifiedLocalization;
import org.firstinspires.ftc.teamcode.Shooter.ShooterCamera;

@TeleOp
public class SensorBot extends LinearOpMode {

    private boolean lockedOn = false;
    private DriveTrain driveTrain;
    private ShooterCamera shooterCamera;
    private int state = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        driveTrain = new DriveTrain(hardwareMap, gamepad1, telemetry);
        shooterCamera = new ShooterCamera(telemetry, hardwareMap);


        waitForStart();


        if (isStopRequested()) {
            return;
        }
        while (opModeIsActive() && !isStopRequested()) {

            driveTrain.drive();

            if (gamepad1.square) {
                shootPrime();
            }

            telemetry.update();


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
