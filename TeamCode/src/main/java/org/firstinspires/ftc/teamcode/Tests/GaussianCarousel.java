package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class GaussianCarousel extends LinearOpMode {

    private SparkFunOTOS otos;
    private DcMotor motor;

    private static final double MAX_X_INCHES = 24.0;  // Adjust this range as needed

    @Override
    public void runOpMode() {
        // Initialize the OTOS sensor
        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        // Initialize the motor
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Configure the OTOS sensor
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);

        // Set sensor offset (adjust for your robot)
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(offset);

        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);

        // Calibrate and reset
        otos.calibrateImu();
        otos.resetTracking();

        telemetry.addLine("OTOS -> Motor Power Mapping");
        telemetry.addLine("Press Start to begin...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get the current position from OTOS
            SparkFunOTOS.Pose2D position = otos.getPosition();

            // Map X position to motor power (-1.0 to 1.0)
            double power = mapXToPower(position.x);

            // Set motor power
            motor.setPower(power);

            // Display telemetry
            telemetry.addLine("=== OTOS READINGS ===");
            telemetry.addData("X Position", "%.2f inches", position.x);
            telemetry.addData("Y Position", "%.2f inches", position.y);
            telemetry.addData("Heading", "%.2f degrees", position.h);

            telemetry.addLine("\n=== MOTOR OUTPUT ===");
            telemetry.addData("Mapped Power", "%.3f", power);
            telemetry.addData("Motor Encoder", motor.getCurrentPosition());

            telemetry.addLine("\n=== MAPPING INFO ===");
            telemetry.addData("X Range", "+/- %.1f inches", MAX_X_INCHES);
            telemetry.addData("Power Range", "-1.0 to 1.0");

            // Reset position if gamepad button pressed
            if (gamepad1.a) {
                otos.resetTracking();
                telemetry.addLine("\n>>> Position Reset! <<<");
            }

            // Stop motor with B button
            if (gamepad1.b) {
                motor.setPower(0);
                telemetry.addLine("\n>>> Motor Stopped! <<<");
            }

            telemetry.addLine("\nA = Reset OTOS | B = Stop Motor");
            telemetry.update();
        }

        // Stop motor when OpMode ends
        motor.setPower(0);
    }

    private double mapXToPower(double x) {
        // Linear mapping: power = x / MAX_X_INCHES
        double power = x / MAX_X_INCHES;

        // Clamp to valid motor power range
        return Range.clip(power, -1.0, 1.0);
    }


}