package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "intake", group = "TeleOp")
public class Intake extends OpMode {
    private DcMotor leftIntake;
    private DcMotor rightIntake;

    @Override
    public void init() {
        leftIntake = hardwareMap.dcMotor.get("left_intake");
        rightIntake = hardwareMap.dcMotor.get("right_intake");

    }
    @Override
    public void loop() {
        leftIntake.setPower(gamepad1.left_stick_x);
        rightIntake.setPower((leftIntake.getPower()));
    }
}
