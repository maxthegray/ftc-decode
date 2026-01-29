package org.firstinspires.ftc.teamcode.threaded.Auto;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * OTOS Calibration Program
 *
 * Use this to calibrate your Sparkfun OTOS sensor's linear and angular scalars.
 *
 * === CALIBRATION PROCEDURE ===
 *
 * LINEAR SCALAR:
 *   1. Place robot at a known starting point (use tape on floor)
 *   2. Press A to reset position
 *   3. Manually push robot EXACTLY 48 inches forward (measure with tape)
 *   4. Read the "Measured Distance" on telemetry
 *   5. Calculate: newLinearScalar = currentScalar * (48.0 / measuredDistance)
 *   6. Use D-pad Up/Down to adjust until measured ≈ 48.0
 *   7. Write down your LINEAR_SCALAR value
 *
 * ANGULAR SCALAR:
 *   1. Place robot with a heading reference (align to field wall)
 *   2. Press A to reset position
 *   3. Manually spin robot EXACTLY 10 full rotations (3600°)
 *   4. Read the "Total Rotation" on telemetry
 *   5. Calculate: newAngularScalar = currentScalar * (3600.0 / measuredDegrees)
 *   6. Use D-pad Left/Right to adjust until measured ≈ 3600°
 *   7. Write down your ANGULAR_SCALAR value
 *
 * === CONTROLS ===
 *   Left Stick  = Drive forward/backward
 *   Right Stick = Rotate
 *
 *   A = Reset position to (0, 0, 0°)
 *   B = Calibrate IMU (keep robot still!)
 *
 *   D-pad Up/Down    = Adjust LINEAR_SCALAR (±0.001)
 *   D-pad Left/Right = Adjust ANGULAR_SCALAR (±0.001)
 *
 *   Left Bumper  = Fine adjust mode (±0.0001)
 *   Right Bumper = Coarse adjust mode (±0.01)
 *
 *   Y = Print final values to copy
 */
@TeleOp(name = "OTOS Calibration", group = "Tuning")
public class OtosCalibration extends LinearOpMode {

    private SparkFunOTOS otos;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // Current calibration values
    private double linearScalar = 1.0;
    private double angularScalar = 1.0;

    // Tracking for calibration
    private double totalRotationDegrees = 0;
    private double lastHeadingDegrees = 0;
    private double totalDistanceInches = 0;
    private double lastX = 0;
    private double lastY = 0;

    // Button edge detection
    private boolean prevA, prevB, prevY;
    private boolean prevDpadUp, prevDpadDown, prevDpadLeft, prevDpadRight;

    // Adjustment increment
    private double adjustIncrement = 0.001;

    @Override
    public void runOpMode() {
        // Initialize OTOS
        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        configureOtos();

        // Initialize motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("=== OTOS CALIBRATION ===");
        telemetry.addLine();
        telemetry.addLine("A = Reset position");
        telemetry.addLine("B = Calibrate IMU");
        telemetry.addLine("D-pad = Adjust scalars");
        telemetry.addLine("Y = Print final values");
        telemetry.addLine();
        telemetry.addLine("See code comments for");
        telemetry.addLine("calibration procedure.");
        telemetry.update();

        waitForStart();

        // Initialize tracking
        resetTracking();

        while (opModeIsActive()) {
            // Read current pose
            SparkFunOTOS.Pose2D pose = otos.getPosition();
            double currentX = pose.x;
            double currentY = pose.y;
            double currentHeadingRad = pose.h;
            double currentHeadingDeg = Math.toDegrees(currentHeadingRad);

            // Update total distance traveled
            double dx = currentX - lastX;
            double dy = currentY - lastY;
            double distanceDelta = Math.sqrt(dx * dx + dy * dy);
            totalDistanceInches += distanceDelta;
            lastX = currentX;
            lastY = currentY;

            // Update total rotation (handle wraparound)
            double headingDelta = currentHeadingDeg - lastHeadingDegrees;
            // Normalize to -180 to 180
            while (headingDelta > 180) headingDelta -= 360;
            while (headingDelta < -180) headingDelta += 360;
            totalRotationDegrees += headingDelta;
            lastHeadingDegrees = currentHeadingDeg;

            // === HANDLE INPUT ===

            // Drive
            double forward = -gamepad1.left_stick_y * 0.5;
            double strafe = -gamepad1.left_stick_x * 0.5;
            double rotate = -gamepad1.right_stick_x * 0.4;
            drive(forward, strafe, rotate);

            // Adjust increment based on bumpers
            if (gamepad1.left_bumper) {
                adjustIncrement = 0.0001;  // Fine
            } else if (gamepad1.right_bumper) {
                adjustIncrement = 0.01;    // Coarse
            } else {
                adjustIncrement = 0.001;   // Normal
            }

            // A = Reset position
            if (gamepad1.a && !prevA) {
                otos.resetTracking();
                resetTracking();
                telemetry.speak("Reset");
            }
            prevA = gamepad1.a;

            // B = Calibrate IMU
            if (gamepad1.b && !prevB) {
                telemetry.addLine(">>> CALIBRATING IMU - KEEP STILL! <<<");
                telemetry.update();
                otos.calibrateImu();
                sleep(500);
                otos.resetTracking();
                resetTracking();
                telemetry.speak("IMU calibrated");
            }
            prevB = gamepad1.b;

            // Y = Print final values
            if (gamepad1.y && !prevY) {
                telemetry.speak("Values printed");
            }
            prevY = gamepad1.y;

            // D-pad Up = Increase linear scalar
            if (gamepad1.dpad_up && !prevDpadUp) {
                linearScalar += adjustIncrement;
                otos.setLinearScalar(linearScalar);
            }
            prevDpadUp = gamepad1.dpad_up;

            // D-pad Down = Decrease linear scalar
            if (gamepad1.dpad_down && !prevDpadDown) {
                linearScalar -= adjustIncrement;
                otos.setLinearScalar(linearScalar);
            }
            prevDpadDown = gamepad1.dpad_down;

            // D-pad Right = Increase angular scalar
            if (gamepad1.dpad_right && !prevDpadRight) {
                angularScalar += adjustIncrement;
                otos.setAngularScalar(angularScalar);
            }
            prevDpadRight = gamepad1.dpad_right;

            // D-pad Left = Decrease angular scalar
            if (gamepad1.dpad_left && !prevDpadLeft) {
                angularScalar -= adjustIncrement;
                otos.setAngularScalar(angularScalar);
            }
            prevDpadLeft = gamepad1.dpad_left;

            // === TELEMETRY ===
            telemetry.addLine("════════ OTOS CALIBRATION ════════");
            telemetry.addLine();

            telemetry.addLine("── CURRENT SCALARS ──");
            telemetry.addData("LINEAR_SCALAR", "%.6f", linearScalar);
            telemetry.addData("ANGULAR_SCALAR", "%.6f", angularScalar);
            telemetry.addData("Adjust Increment", "%.4f (LB=fine, RB=coarse)", adjustIncrement);

            telemetry.addLine();
            telemetry.addLine("── CURRENT POSITION ──");
            telemetry.addData("X", "%.2f in", currentX);
            telemetry.addData("Y", "%.2f in", currentY);
            telemetry.addData("Heading", "%.2f°", currentHeadingDeg);

            telemetry.addLine();
            telemetry.addLine("── LINEAR CALIBRATION ──");
            telemetry.addData("Total Distance", "%.2f in", totalDistanceInches);
            telemetry.addData("Straight Line", "%.2f in", Math.sqrt(currentX*currentX + currentY*currentY));
            telemetry.addLine("(Push robot 48\" and adjust until this reads 48.0)");

            telemetry.addLine();
            telemetry.addLine("── ANGULAR CALIBRATION ──");
            telemetry.addData("Total Rotation", "%.1f°", Math.abs(totalRotationDegrees));
            telemetry.addData("Full Spins", "%.2f", Math.abs(totalRotationDegrees) / 360.0);
            telemetry.addLine("(Spin robot 10x and adjust until this reads 3600°)");

            telemetry.addLine();
            telemetry.addLine("── COPY THESE VALUES ──");
            telemetry.addLine(String.format("otos.setLinearScalar(%.6f);", linearScalar));
            telemetry.addLine(String.format("otos.setAngularScalar(%.6f);", angularScalar));

            telemetry.addLine();
            telemetry.addLine("A=Reset  B=CalibrateIMU  D-pad=Adjust");

            telemetry.update();
        }

        // Stop motors
        drive(0, 0, 0);
    }

    private void configureOtos() {
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

        // No offset (sensor is centered)
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(offset);

        // Start with 1.0 scalars
        otos.setLinearScalar(linearScalar);
        otos.setAngularScalar(angularScalar);

        // Calibrate and reset
        otos.calibrateImu();
        otos.resetTracking();
    }

    private void resetTracking() {
        SparkFunOTOS.Pose2D pose = otos.getPosition();
        lastX = pose.x;
        lastY = pose.y;
        lastHeadingDegrees = Math.toDegrees(pose.h);
        totalDistanceInches = 0;
        totalRotationDegrees = 0;
    }

    private void drive(double forward, double strafe, double rotate) {
        double fl = forward + strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
}