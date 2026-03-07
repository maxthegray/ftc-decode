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
        lift1.setDirection(DcMotorSimple.Direction.FORWARD); //set these
        lift2 = hardwareMap.get(CRServo.class, "lift2");
        lift2.setDirection(DcMotorSimple.Direction.FORWARD); //set these
        lift3 = hardwareMap.get(CRServo.class, "lift3");
        lift3.setDirection(DcMotorSimple.Direction.FORWARD); //set these
    }

    @Override
    public void loop() {
        if (!gamepad1.cross) {
            lift1.setPower(0);
            lift2.setPower(0);
            lift3.setPower(0);
            telemetry.addLine("Cross for Control");
            telemetry.update();
            return;
        }

        double power = 0;
        double lift3Power = 0;
        if (gamepad1.right_bumper) {
            power = 1.0;
            lift3Power = gamepad1.left_bumper ? 0.5 : 1.0;
        }

        lift1.setPower(power);
        lift2.setPower(power);
        lift3.setPower(lift3Power);

        telemetry.addData("lift1/2 power", power);
        telemetry.addData("lift3 power",   lift3Power);
        telemetry.update();
    }

    @Override
    public void stop() {
        lift1.setPower(0);
        lift2.setPower(0);
        lift3.setPower(0);
    }
}
