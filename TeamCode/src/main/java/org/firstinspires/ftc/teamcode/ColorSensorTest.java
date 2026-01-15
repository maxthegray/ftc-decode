package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name="ColorSensorTest")

public class ColorSensorTest extends OpMode {
    private RevColorSensorV3 a;

    public void init() {
        a = hardwareMap.get(RevColorSensorV3.class, "intake_color2");

    }
    public void loop() {


        telemetry();
    }
    void telemetry() {

        a.setGain(a.getGain() + gamepad1.left_stick_y);

        telemetry.addData("Alpha", a.alpha());
        telemetry.addData("gain", a.getGain());

        telemetry.addData("red", a.red());
        telemetry.addData("green", a.green());
        telemetry.addData("blue", a.blue());

        telemetry.update();
    }

    private int rgbToHue(int r, int g, int b) {
        float[] hsv = new float[3];

        android.graphics.Color.RGBToHSV(r, g, b, hsv);
        return (int) hsv[0];
    }

}
