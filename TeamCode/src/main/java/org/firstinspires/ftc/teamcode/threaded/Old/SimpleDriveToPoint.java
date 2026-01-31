package org.firstinspires.ftc.teamcode.threaded.Old;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SimpleDriveToPoint {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private SparkFunOTOS otos;

    // Tuning
    public static double DRIVE_POWER = 0.4;
    public static double TURN_POWER = 0.3;
    public static double POSITION_TOLERANCE = 1.0;  // inches
    public static double HEADING_TOLERANCE = 3.0;   // degrees

    public SimpleDriveToPoint(HardwareMap hardwareMap) {
        // Motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // OTOS
        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setOffset(new SparkFunOTOS.Pose2D(0, 0, -90));
        otos.calibrateImu();
        otos.resetTracking();
    }

    /**
     * Drive to a point. Call this in a loop.
     * Returns true when at target.
     */
    public boolean update(double targetX, double targetY, double targetHeading) {
        SparkFunOTOS.Pose2D pose = otos.getPosition();

        // Calculate errors
        double xError = targetX - pose.x;
        double yError = targetY - pose.y;
        double headingError = normalizeAngle(targetHeading - pose.h);

        double distance = Math.sqrt(xError * xError + yError * yError);

        // Check if we're done
        if (distance < POSITION_TOLERANCE && Math.abs(headingError) < HEADING_TOLERANCE) {
            stop();
            return true;
        }

        // Convert to robot-relative
        double robotAngle = Math.toRadians(pose.h);
        double forward = xError * Math.cos(robotAngle) + yError * Math.sin(robotAngle);
        double strafe = -xError * Math.sin(robotAngle) + yError * Math.cos(robotAngle);

        // Normalize to unit vector and apply power
        if (distance > 0.001) {
            forward = (forward / distance) * DRIVE_POWER;
            strafe = (strafe / distance) * DRIVE_POWER;
        }

        // Simple turn - just direction, fixed power
        double turn = 0;
        if (Math.abs(headingError) > HEADING_TOLERANCE) {
            turn = headingError > 0 ? TURN_POWER : -TURN_POWER;
        }

        // Mecanum drive
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        // Normalize
        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
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

        return false;
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public SparkFunOTOS.Pose2D getPosition() {
        return otos.getPosition();
    }

    public void resetPosition() {
        otos.resetTracking();
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}