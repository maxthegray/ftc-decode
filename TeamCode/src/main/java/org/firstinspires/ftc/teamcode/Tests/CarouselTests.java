package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.threaded.Robot;

@TeleOp(name = "CarouselTest", group = "Testing")
public class CarouselTests extends OpMode {

    private Robot r;

    private boolean flickerUp = false;
    private boolean lastButtonState = false;
    private static final double down = 0;
    private static final double up = 0.4;

    @Override
    public void init() {
        r = new Robot(hardwareMap);
        r.init();
    }

    @Override
    public void loop() {
        r.carouselMotor.setPower(gamepad1.left_stick_x);

        if (gamepad1.squareWasPressed()) {
            flickerUp = !flickerUp;
            r.kicker.setPosition(flickerUp ? up : down);
        }

        telemetry.addData("Motor Power", gamepad1.left_stick_x);
        telemetry.addData("Flicker", flickerUp ? "UP" : "DOWN");
        telemetry.addData("Red", r.blColor.red());
        telemetry.addData("Green", r.blColor.green());
        telemetry.addData("Blue", r.brColor.blue());


        telemetry.update();
    }
}