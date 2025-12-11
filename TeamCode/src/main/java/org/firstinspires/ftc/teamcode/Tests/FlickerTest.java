package org.firstinspires.ftc.teamcode.Tests;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

//2000 ticks per 1 rotation


@TeleOp(name = "Flicker", group = "Testing")
public class FlickerTest extends OpMode {
    Servo servo;
    double position;
    double down = 0;
    double up = 0.4; //measured


    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "flickServo");
        position = 0;


    }

    @Override
    public void loop() {
        servo.setPosition(position);
        telemetry.addData("servo position:", position);

        if (gamepad1.dpad_up) {
            position = up;
            sleep(50);
        }
        if (gamepad1.dpad_down) {
            position = down;
            sleep(50);
        }
        telemetry.update();


    }

}
