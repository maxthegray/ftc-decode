package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Flicker", group = "Testing")
public class FlickerTest extends OpMode {
    Servo servo;
    double position;

    double down = 0;
    double up = 1;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "flickerServo");
        position = down;
    }

    @Override
    public void loop() {
        servo.setPosition(position);
        telemetry.addData("servo position:", position);

        if (gamepad1.dpad_up) {
            position = up;
            sleep(100);
        }
        if (gamepad1.dpad_down) {
            position = down;
            sleep(100);
        }
        telemetry.update();


    }

}
