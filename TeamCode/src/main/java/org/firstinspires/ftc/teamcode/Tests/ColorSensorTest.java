package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp(name = "Color Sensor Calibration", group = "Calibration")
public class ColorSensorTest extends LinearOpMode {

    private RevColorSensorV3 intakeA, intakeB;
    private RevColorSensorV3 backLeftA, backLeftB;
    private RevColorSensorV3 backRightA, backRightB;

    @Override
    public void runOpMode() {
        // Initialize sensors
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
            // Intake position
            telemetry.addLine("=== INTAKE ===");
            telemetry.addData("A: alpha", intakeA.alpha());
            telemetry.addData("A: blue/green", String.format("%.2f", ratio(intakeA)));

            telemetry.addData("B: alpha", intakeB.alpha());
            telemetry.addData("B: blue/green", String.format("%.2f", ratio(intakeB)));

            // Back Left position
            telemetry.addLine("=== BACK LEFT ===");
            telemetry.addData("A: alpha", backLeftA.alpha());
            telemetry.addData("A: blue/green", String.format("%.2f", ratio(backLeftA)));

            telemetry.addData("B: alpha", backLeftB.alpha());
            telemetry.addData("B: blue/green", String.format("%.2f", ratio(backLeftB)));

            // Back Right position
            telemetry.addLine("=== BACK RIGHT ===");
            telemetry.addData("A: alpha", backRightA.alpha());
            telemetry.addData("A: blue/green", String.format("%.2f", ratio(backRightA)));

            telemetry.addData("B: alpha", backRightB.alpha());
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