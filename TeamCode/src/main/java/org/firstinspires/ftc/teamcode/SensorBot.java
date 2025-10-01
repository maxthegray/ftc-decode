package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Localization.UnifiedLocalization;
import org.firstinspires.ftc.teamcode.Shooter.ShooterCamera;

@TeleOp
public class SensorBot extends LinearOpMode {

    private boolean lockedOn = false;
    private DriveTrain driveTrain;
    private UnifiedLocalization location;
    private ShooterCamera shooterCamera;

    @Override
    public void runOpMode() throws InterruptedException {

        location = new UnifiedLocalization(telemetry, hardwareMap);
        driveTrain = new DriveTrain(hardwareMap, gamepad1, telemetry);
        shooterCamera = new ShooterCamera(telemetry, hardwareMap);

        while (opModeIsActive()) {
            driveTrain.drive();
            handleControls();}
        }

    private void handleControls() throws InterruptedException {
        if (gamepad1.dpad_down) {
            driveTrain.goTo(0, 0, 0.5);
        }

        if (gamepad1.dpad_up) {
            driveTrain.rampToXYH(0, 0, 0, 1);
        }

        // toggle lock
        if (gamepad1.a) {
            if (lockedOn) {
                driveTrain.unlockFromTag();
            } else {
                driveTrain.lockOntoTag();
            }
            lockedOn = !lockedOn;
            sleep(50); // debounce
        }
    }


}
