package org.firstinspires.ftc.teamcode.threaded;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Controls intake motors and servo.
 */
public class IntakeController {

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private final CRServo servo;

    private static final double MOTOR_POWER = 1.0;
    private static final double SERVO_POWER = 1.0;

    private boolean running = false;

    public IntakeController(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotorEx.class, "left_intake");
        rightMotor = hardwareMap.get(DcMotorEx.class, "right_intake");
        servo = hardwareMap.get(CRServo.class, "intake_servo");

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void forward() { forward(0.55); }

    public void forward(double scale) {
        double s = Math.max(0, Math.min(1, scale));
        leftMotor.setPower(-MOTOR_POWER * s);
        rightMotor.setPower(-MOTOR_POWER * s);
        servo.setPower(-SERVO_POWER * s);
        running = true;
    }

    public void reverse() { reverse(0.55); }

    public void reverse(double scale) {
        double s = Math.max(0, Math.min(1, scale));
        leftMotor.setPower(MOTOR_POWER * s);
        rightMotor.setPower(MOTOR_POWER * s);
        servo.setPower(SERVO_POWER * s);
        running = true;
    }

    public void stop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        servo.setPower(0);
        running = false;
    }

    public boolean isRunning() {
        return running;
    }
}