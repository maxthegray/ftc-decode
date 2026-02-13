package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp
public class TickTurnFinder extends LinearOpMode {

    private DcMotor carousel;

    @Override
    public void runOpMode() {

        carousel = hardwareMap.get(DcMotor.class, "carousel_motor");

        // Encoder setup
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Optional but usually correct for carousels
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        while (opModeIsActive()) {

            // Use left stick Y to spin carousel
            double power = -gamepad1.left_stick_y;

            // Deadzone
            if (Math.abs(power) < 0.05) {
                power = 0;
            }

            carousel.setPower(power);

            // Telemetry
            telemetry.addData("Carousel Power", "%.2f", power);
            telemetry.addData("Encoder Ticks", carousel.getCurrentPosition());
            telemetry.addData("Is Busy", carousel.isBusy());
            telemetry.update();
        }

        carousel.setPower(0);
    }
}
