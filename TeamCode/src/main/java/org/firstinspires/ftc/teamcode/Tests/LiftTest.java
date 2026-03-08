package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class LiftTest extends OpMode {

    private CRServo lift1, lift2, lift3;


    @Override
    public void init() {
        lift1 = hardwareMap.get(CRServo.class, "lift1");
        lift1.setDirection(DcMotorSimple.Direction.REVERSE); //set these
        lift2 = hardwareMap.get(CRServo.class, "lift2");
        lift2.setDirection(DcMotorSimple.Direction.REVERSE); //set these
        lift3 = hardwareMap.get(CRServo.class, "lift3");
        lift3.setDirection(DcMotorSimple.Direction.FORWARD); //set these
    }
    //lift1 opposite of empty
    // lift2 backleft
    //liftthree right way
    @Override
    public void loop() {
        boolean all = gamepad1.dpad_up;
        lift1.setPower(all || gamepad1.triangle ? 0.2 : 0);
        lift2.setPower(all || gamepad1.square   ? 0.5 : 0);
        lift3.setPower(all || gamepad1.circle   ? 0.4 : 0);

        telemetry.addData("lift1 (triangle)", gamepad1.triangle);
        telemetry.addData("lift2 (square)",   gamepad1.square);
        telemetry.addData("lift3 (circle)",   gamepad1.circle);
        telemetry.update();
    }

    @Override
    public void stop() {
        lift1.setPower(0);
        lift2.setPower(0);
        lift3.setPower(0);
    }
}
