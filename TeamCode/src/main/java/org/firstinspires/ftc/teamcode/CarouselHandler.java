package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import java.util.Collections;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp(name = "Carousel Handler", group = "TeleOp")
public class CarouselHandler extends LinearOpMode {

    private RevColorSensorV3 colorSensor;

    private List <Integer> slots = new ArrayList<>(Arrays.asList(0,0,0));
    private int rotationDegrees =0;
    private double threshold = 80;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.a) {
                rotateToNext();
                sleep(300);
            }

            if (gamepad1.b) {
                rotateToPrevious();
                sleep(300);
            }

            if (gamepad1.x) {
                assignColorToCurrentSlot();
                sleep(300);
            }

            displaySlots();
        }
    }

    public void rotateToNext() {
        Collections.rotate(slots, -1);
        rotationDegrees-=120;
    }

    public void rotateToPrevious() {
        Collections.rotate(slots,1);
        rotationDegrees+=120;
    }

    private void assignColorToCurrentSlot() {
        int color = detectColor();
        slots.set(0,color);
    }

    private int detectColor() {
        int red = colorSensor.getNormalizedColors().toColor();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        if (blue > threshold || green > threshold) {
            if (green > blue) {
                return 2;
            } else {
                return 1;
            }
        }
        return 0;
    }

    private void displaySlots() {
        telemetry.addData("slots", "[%d, %d, %d]", slots.get(0), slots.get(1), slots.get(2));
        telemetry.addData("intake slot", 0);
        telemetry.addData("rotation (degrees)", rotationDegrees);
        telemetry.addData("color sensor readings", "R: %d, G: %d, B: %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
        telemetry.update();
    }
}