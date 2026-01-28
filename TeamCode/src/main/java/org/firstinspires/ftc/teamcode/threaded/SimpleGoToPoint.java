package org.firstinspires.ftc.teamcode.threaded;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Dead simple go-to-point with OTOS.
 *
 * Usage:
 *   SimpleGoToPoint goTo = new SimpleGoToPoint(hardwareMap);
 *   goTo.goToPoint(24, 24, 90);
 *   while (!goTo.isDone()) { goTo.update(); }
 */
public class SimpleGoToPoint {

    private SparkFunOTOS otos;
    private DcMotor fl, fr, bl, br;

    // Tuning
    public static double DRIVE_SPEED = 0.4;
    public static double TURN_SPEED = 0.3;
    public static double POSITION_TOLERANCE = 5.0;
    public static double HEADING_TOLERANCE = 3.0;

    private double targetX, targetY, targetH;
    private boolean running = false;

    public SimpleGoToPoint(HardwareMap hw) {
        otos = hw.get(SparkFunOTOS.class, "sensor_otos");
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.calibrateImu();
        otos.resetTracking();

        fl = hw.get(DcMotor.class, "frontLeftMotor");
        fr = hw.get(DcMotor.class, "frontRightMotor");
        bl = hw.get(DcMotor.class, "backLeftMotor");
        br = hw.get(DcMotor.class, "backRightMotor");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void goToPoint(double x, double y, double heading) {
        targetX = x;
        targetY = y;
        targetH = heading;
        running = true;
    }

    public void update() {
        if (!running) return;

        SparkFunOTOS.Pose2D pose = otos.getPosition();

        double errorX = targetX - pose.x;
        double errorY = targetY - pose.y;
        double errorH = targetH - pose.h;

        while (errorH > 180) errorH -= 360;
        while (errorH < -180) errorH += 360;

        double distance = Math.sqrt(errorX * errorX + errorY * errorY);

        if (distance < POSITION_TOLERANCE && Math.abs(errorH) < HEADING_TOLERANCE) {
            stop();
            return;
        }

        double headingRad = Math.toRadians(pose.h);
        double forward = (errorY * Math.cos(headingRad) + errorX * Math.sin(headingRad));
        double strafe = (errorX * Math.cos(headingRad) - errorY * Math.sin(headingRad));

        double max = Math.max(Math.abs(forward), Math.abs(strafe));
        if (max > 0) {
            forward = (forward / max) * DRIVE_SPEED;
            strafe = (strafe / max) * DRIVE_SPEED;
        }

        double turn = 0;
        if (Math.abs(errorH) > HEADING_TOLERANCE) {
            turn = (errorH > 0) ? TURN_SPEED : -TURN_SPEED;
        }

        fl.setPower(forward + strafe + turn);
        fr.setPower(forward - strafe - turn);
        bl.setPower(forward - strafe + turn);
        br.setPower(forward + strafe - turn);
    }

    public boolean isDone() { return !running; }

    public void stop() {
        running = false;
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public double getX() { return otos.getPosition().x; }
    public double getY() { return otos.getPosition().y; }
    public double getHeading() { return otos.getPosition().h; }

    public void setPosition(double x, double y, double h) {
        otos.setPosition(new SparkFunOTOS.Pose2D(x, y, h));
    }
}