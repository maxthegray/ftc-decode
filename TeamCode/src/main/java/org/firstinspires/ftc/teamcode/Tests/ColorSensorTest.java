package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//@Disabled
@TeleOp(name = "Color Sensor Calibration", group = "Calibration")
public class ColorSensorTest extends LinearOpMode {

    private RevColorSensorV3 intakeA, intakeB;
    private RevColorSensorV3 backLeftA, backLeftB;
    private RevColorSensorV3 backRightA, backRightB;

    @Override
    public void runOpMode() {
        intakeA = hardwareMap.get(RevColorSensorV3.class, "intake_color1");
        intakeB = hardwareMap.get(RevColorSensorV3.class, "intake_color2");
        backLeftA = hardwareMap.get(RevColorSensorV3.class, "BL_color");
        backLeftB = hardwareMap.get(RevColorSensorV3.class, "BL_upper");
        backRightA = hardwareMap.get(RevColorSensorV3.class, "BR_color");
        backRightB = hardwareMap.get(RevColorSensorV3.class, "BR_upper");

        telemetry.addLine("Color Sensor Calibration");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("=== INTAKE ===");
            telemetry.addData("A: dist (mm)", String.format("%.1f", intakeA.getDistance(DistanceUnit.MM)));
            telemetry.addData("A: blue/green", String.format("%.2f", ratio(intakeA)));

            telemetry.addData("B: dist (mm)", String.format("%.1f", intakeB.getDistance(DistanceUnit.MM)));
            telemetry.addData("B: blue/green", String.format("%.2f", ratio(intakeB)));

            telemetry.addLine("=== BACK LEFT ===");
            telemetry.addData("A: dist (mm)", String.format("%.1f", backLeftA.getDistance(DistanceUnit.MM)));
            telemetry.addData("A: blue/green", String.format("%.2f", ratio(backLeftA)));

            telemetry.addData("B: dist (mm)", String.format("%.1f", backLeftB.getDistance(DistanceUnit.MM)));
            telemetry.addData("B: blue/green", String.format("%.2f", ratio(backLeftB)));

            telemetry.addLine("=== BACK RIGHT ===");
            telemetry.addData("A: dist (mm)", String.format("%.1f", backRightA.getDistance(DistanceUnit.MM)));
            telemetry.addData("A: blue/green", String.format("%.2f", ratio(backRightA)));

            telemetry.addData("B: dist (mm)", String.format("%.1f", backRightB.getDistance(DistanceUnit.MM)));
            telemetry.addData("B: blue/green", String.format("%.2f", ratio(backRightB)));

            telemetry.update();
        }
    }

    private double ratio(RevColorSensorV3 sensor) {
        int green = sensor.green();
        if (green == 0) return 0;
        return (double) sensor.blue() / green;
    }
}