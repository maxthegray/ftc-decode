package org.firstinspires.ftc.teamcode.Tests;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.R;

@TeleOp(name = "CarouselTest", group = "Testing")
public class CarouselTests extends OpMode {
    DcMotor carouselMotor;
    double motorPosition;

    private DigitalChannel leftFinLimit;
    private DigitalChannel rightFinLimit;

    @Override
    public void init() {
       carouselMotor = hardwareMap.dcMotor.get("carouselMotor");
       carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       leftFinLimit = hardwareMap.get(DigitalChannel.class, "leftFin");
       leftFinLimit.setMode(DigitalChannel.Mode.INPUT);

       rightFinLimit = hardwareMap.get(DigitalChannel.class, "rightFin");
       rightFinLimit.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void loop() {
        motorPosition = carouselMotor.getCurrentPosition();
        carouselMotor.setPower(gamepad1.left_stick_x);

        if (gamepad1.dpad_left) {
            rotate(-1);
            sleep(500);
        }
        if (gamepad1.dpad_right) {
            rotate(1);
            sleep(500);
        }

        telemetry.addData("Carousel Motor Power", carouselMotor.getPower());
        telemetry.addData("Carousel Motor Position", carouselMotor.getCurrentPosition());

        telemetry.addData("Left Fin", leftFinLimit.getState() ? "P" : "MP");
        telemetry.addData("Right Fin", rightFinLimit.getState() ? "P" : "NP");

        telemetry.update();
    }

    public void rotate(int LR) {
        carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carouselMotor.setTargetPosition(carouselMotor.getCurrentPosition() + LR * 2000);



    }
}

