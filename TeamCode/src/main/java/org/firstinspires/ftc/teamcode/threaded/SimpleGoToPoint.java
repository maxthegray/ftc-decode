package org.firstinspires.ftc.teamcode.threaded;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Simple go-to-point with OTOS and PID.
 *
 * Usage:
 *   SimpleGoToPoint goTo = new SimpleGoToPoint(hardwareMap);
 *   goTo.goToPoint(24, 24, 90);
 *   while (!goTo.isDone()) { goTo.update(); }
 */
public class SimpleGoToPoint {

    private SparkFunOTOS otos;
    private DcMotor fl, fr, bl, br;
    private ElapsedTime timer = new ElapsedTime();

    // Drive PID
    public static double DRIVE_P = 0.05;
    public static double DRIVE_I = 0.0;
    public static double DRIVE_D = 0;

    // Turn PID
    public static double TURN_P = 0.02;
    public static double TURN_I = 0.0;
    public static double TURN_D = 0.0;

    // Limits
    public static double MAX_DRIVE = 0.6;
    public static double MAX_TURN = 0.4;

    // Tolerances
    public static double POSITION_TOLERANCE = 3.0;
    public static double HEADING_TOLERANCE = 2.0;

    // Target
    private double targetX, targetY, targetH;
    private boolean running = false;

    // PID state
    private double lastErrorX, lastErrorY, lastErrorH;
    private double integralX, integralY, integralH;
    private double lastTime;

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

        // Reset PID
        lastErrorX = 0;
        lastErrorY = 0;
        lastErrorH = 0;
        integralX = 0;
        integralY = 0;
        integralH = 0;
        timer.reset();
        lastTime = 0;
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

        // Time delta
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        if (dt <= 0) dt = 0.02;

        // Drive PID
        integralX += errorX * dt;
        integralY += errorY * dt;
        double derivX = (errorX - lastErrorX) / dt;
        double derivY = (errorY - lastErrorY) / dt;

        double powerX = DRIVE_P * errorX + DRIVE_I * integralX + DRIVE_D * derivX;
        double powerY = DRIVE_P * errorY + DRIVE_I * integralY + DRIVE_D * derivY;

        lastErrorX = errorX;
        lastErrorY = errorY;

        // Turn PID
        integralH += errorH * dt;
        double derivH = (errorH - lastErrorH) / dt;
        double turn = TURN_P * errorH + TURN_I * integralH + TURN_D * derivH;
        lastErrorH = errorH;

        // Clamp
        powerX = Math.max(-MAX_DRIVE, Math.min(MAX_DRIVE, powerX));
        powerY = Math.max(-MAX_DRIVE, Math.min(MAX_DRIVE, powerY));
        turn = Math.max(-MAX_TURN, Math.min(MAX_TURN, turn));

        // Field to robot centric
        double headingRad = Math.toRadians(pose.h);
        double forward = powerY * Math.cos(headingRad) + powerX * Math.sin(headingRad);
        double strafe = powerX * Math.cos(headingRad) - powerY * Math.sin(headingRad);

        // Mecanum
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