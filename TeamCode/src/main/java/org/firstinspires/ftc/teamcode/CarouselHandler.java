package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Carousel Handler", group = "TeleOp")
public class CarouselHandler extends LinearOpMode {

    private RevColorSensorV3 colorSensor;

    private int[] slots = {0, 0, 0};
    private int currentSlot = 0;
    private int rotationDegrees = currentSlot * 120;
    private double threshold = 0.1;

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
        currentSlot = (currentSlot + 1) % 3;
    }

    public void rotateToPrevious() {
        currentSlot = (currentSlot - 1 + 3) % 3;
    }

    private void assignColorToCurrentSlot() {
        int color = detectColor();
        slots[currentSlot] = color;
    }

    private int detectColor() {
        int red = colorSensor.getNormalizedColors().toColor();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        if (red > threshold || green > threshold) {
            if (green > red) {
                return 2;
            } else {
                return 1;
            }
        }
        return 0;
    }

    private void displaySlots() {
        telemetry.addData("slots", "[%d, %d, %d]", slots[0], slots[1], slots[2]);
        telemetry.addData("intake slot", currentSlot);
        telemetry.addData("rotation (degrees)", rotationDegrees);
        telemetry.addData("color sensor readings", "R: %d, G: %d, B: %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
        telemetry.update();
    }
}