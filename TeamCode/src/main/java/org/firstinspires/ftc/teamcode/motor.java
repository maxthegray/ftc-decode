package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Light", group = "Testing")
public class motor extends OpMode {
    Servo servo;
    double position;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
        position = 0;
    }

    @Override
    public void loop() {
        servo.setPosition(position);
        telemetry.addData("servo position:", position);

        if (gamepad1.dpad_up) {
            position = 1;
            sleep(100);
        }
        if (gamepad1.dpad_down) {
            position = 0;
            sleep(100);
        }
        telemetry.update();


    }

}
