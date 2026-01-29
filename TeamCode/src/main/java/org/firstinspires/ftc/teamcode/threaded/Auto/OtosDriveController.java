package org.firstinspires.ftc.teamcode.threaded.Auto;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * OTOS-based drive controller for autonomous.
 * Uses three PID loops (X, Y, Heading) with field-centric mecanum control.
 *
 * Usage:
 *   OtosDriveController drive = new OtosDriveController(hardwareMap);
 *   drive.setStartPose(0, 0, 0);  // Set initial position
 *
 *   drive.driveTo(24, 0, 0);      // Drive to position
 *   while (opModeIsActive() && !drive.isAtTarget()) {
 *       drive.update();
 *   }
 *   drive.stop();
 */
public class OtosDriveController {

    // ========================= HARDWARE =========================
    private final SparkFunOTOS otos;
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // ========================= PID TUNING (public for Dashboard) =========================
    // Linear (X/Y) PID
    public static double LINEAR_P = 0.05;
    public static double LINEAR_I = 0.0;
    public static double LINEAR_D = 0.005;

    // Heading PID
    public static double HEADING_P = 0.8;
    public static double HEADING_I = 0.0;
    public static double HEADING_D = 0.05;

    // Tolerances
    public static double POSITION_TOLERANCE = 1.0;    // inches
    public static double HEADING_TOLERANCE = 2.0;     // degrees

    // Max speeds
    public static double MAX_LINEAR_SPEED = 0.7;
    public static double MAX_TURN_SPEED = 0.5;

    // Settle time - how long to be within tolerance before declaring "at target"
    public static long SETTLE_TIME_MS = 150;

    // ========================= STATE =========================
    private double targetX = 0;
    private double targetY = 0;
    private double targetHeading = 0;  // radians

    // Current pose (updated each loop)
    private double currentX = 0;
    private double currentY = 0;
    private double currentHeading = 0;  // radians

    // PID state for X
    private double xIntegral = 0;
    private double xLastError = 0;

    // PID state for Y
    private double yIntegral = 0;
    private double yLastError = 0;

    // PID state for Heading
    private double headingIntegral = 0;
    private double headingLastError = 0;

    // Timing
    private final ElapsedTime pidTimer = new ElapsedTime();
    private final ElapsedTime settleTimer = new ElapsedTime();
    private boolean wasInTolerance = false;

    // ========================= CONSTRUCTOR =========================
    public OtosDriveController(HardwareMap hardwareMap) {
        // Initialize OTOS
        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        configureOtos();

        // Initialize motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        // Set directions
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run without encoders (OTOS handles localization)
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidTimer.reset();
    }

    // ========================= OTOS CALIBRATION =========================
    // Run OtosCalibration OpMode and put your values here!
    public static double LINEAR_SCALAR = 1.0;   // Calibrate by pushing robot 48"
    public static double ANGULAR_SCALAR = 1.0;  // Calibrate by spinning robot 10x

    // OTOS offset from robot center (inches). Positive X = forward, Positive Y = left
    public static double OTOS_OFFSET_X = 0.0;
    public static double OTOS_OFFSET_Y = 0.0;
    public static double OTOS_OFFSET_HEADING = 0.0;  // degrees, if sensor is rotated

    private void configureOtos() {
        // Set units
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

        // Set offset (if OTOS is not centered on robot)
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(
                OTOS_OFFSET_X,
                OTOS_OFFSET_Y,
                Math.toRadians(OTOS_OFFSET_HEADING)
        );
        otos.setOffset(offset);

        // Apply calibrated scalars
        otos.setLinearScalar(LINEAR_SCALAR);
        otos.setAngularScalar(ANGULAR_SCALAR);

        // Calibrate IMU
        otos.calibrateImu();

        // Reset tracking
        otos.resetTracking();
    }

    // ========================= POSE SETTING =========================

    /**
     * Set the starting pose. Call this at the beginning of auto.
     */
    public void setStartPose(double x, double y, double headingDegrees) {
        SparkFunOTOS.Pose2D pose = new SparkFunOTOS.Pose2D(x, y, Math.toRadians(headingDegrees));
        otos.setPosition(pose);

        this.currentX = x;
        this.currentY = y;
        this.currentHeading = Math.toRadians(headingDegrees);

        this.targetX = x;
        this.targetY = y;
        this.targetHeading = Math.toRadians(headingDegrees);
    }

    // ========================= MOVEMENT COMMANDS =========================

    /**
     * Drive to an absolute field position.
     * @param x Target X position (inches)
     * @param y Target Y position (inches)
     * @param headingDegrees Target heading (degrees)
     */
    public void driveTo(double x, double y, double headingDegrees) {
        this.targetX = x;
        this.targetY = y;
        this.targetHeading = Math.toRadians(headingDegrees);
        resetPID();
    }

    /**
     * Turn to an absolute heading.
     * @param headingDegrees Target heading (degrees)
     */
    public void turnTo(double headingDegrees) {
        this.targetHeading = Math.toRadians(headingDegrees);
        resetPID();
    }

    /**
     * Drive forward relative to current position and heading.
     * @param inches Distance to drive forward
     */
    public void driveForward(double inches) {
        updatePose();
        double dx = inches * Math.cos(currentHeading);
        double dy = inches * Math.sin(currentHeading);
        driveTo(currentX + dx, currentY + dy, Math.toDegrees(currentHeading));
    }

    /**
     * Strafe right relative to current position.
     * @param inches Distance to strafe (positive = right)
     */
    public void strafeRight(double inches) {
        updatePose();
        double dx = inches * Math.cos(currentHeading - Math.PI / 2);
        double dy = inches * Math.sin(currentHeading - Math.PI / 2);
        driveTo(currentX + dx, currentY + dy, Math.toDegrees(currentHeading));
    }

    /**
     * Strafe left relative to current position.
     * @param inches Distance to strafe (positive = left)
     */
    public void strafeLeft(double inches) {
        strafeRight(-inches);
    }

    // ========================= UPDATE LOOP =========================

    /**
     * Call this every loop iteration.
     * Reads OTOS, calculates PID outputs, and drives motors.
     */
    public void update() {
        // Read current pose from OTOS
        updatePose();

        // Calculate time delta
        double dt = pidTimer.seconds();
        pidTimer.reset();

        // Clamp dt to avoid spikes
        if (dt > 0.1) dt = 0.1;
        if (dt <= 0) dt = 0.01;

        // Calculate errors in field frame
        double xError = targetX - currentX;
        double yError = targetY - currentY;
        double headingError = normalizeAngle(targetHeading - currentHeading);

        // Calculate PID outputs (field frame)
        double xPower = calculatePID(xError, dt, LINEAR_P, LINEAR_I, LINEAR_D,
                xIntegral, xLastError, true);
        double yPower = calculatePID(yError, dt, LINEAR_P, LINEAR_I, LINEAR_D,
                yIntegral, yLastError, false);
        double turnPower = calculatePID(headingError, dt, HEADING_P, HEADING_I, HEADING_D,
                headingIntegral, headingLastError, false);

        // Update PID state
        xIntegral += xError * dt;
        yIntegral += yError * dt;
        headingIntegral += headingError * dt;
        xLastError = xError;
        yLastError = yError;
        headingLastError = headingError;

        // Clamp linear speeds
        xPower = clamp(xPower, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        yPower = clamp(yPower, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        turnPower = clamp(turnPower, -MAX_TURN_SPEED, MAX_TURN_SPEED);

        // Convert field-centric to robot-centric
        double[] robotPowers = fieldToRobot(xPower, yPower, currentHeading);
        double robotX = robotPowers[0];  // forward/backward
        double robotY = robotPowers[1];  // strafe

        // Apply mecanum kinematics
        double fl = robotX + robotY + turnPower;
        double fr = robotX - robotY - turnPower;
        double bl = robotX - robotY + turnPower;
        double br = robotX + robotY - turnPower;

        // Normalize if any motor > 1.0
        double maxPower = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (maxPower > 1.0) {
            fl /= maxPower;
            fr /= maxPower;
            bl /= maxPower;
            br /= maxPower;
        }

        // Set motor powers
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);

        // Update settle timer
        boolean inTolerance = isWithinTolerance();
        if (inTolerance && !wasInTolerance) {
            settleTimer.reset();
        }
        wasInTolerance = inTolerance;
    }

    private void updatePose() {
        SparkFunOTOS.Pose2D pose = otos.getPosition();
        currentX = pose.x;
        currentY = pose.y;
        currentHeading = pose.h;
    }

    private double calculatePID(double error, double dt, double kP, double kI, double kD,
                                double integral, double lastError, boolean isX) {
        double p = kP * error;
        double i = kI * integral;
        double d = (dt > 0) ? kD * (error - lastError) / dt : 0;
        return p + i + d;
    }

    /**
     * Convert field-centric X/Y velocities to robot-centric.
     */
    private double[] fieldToRobot(double fieldX, double fieldY, double robotHeading) {
        double cos = Math.cos(-robotHeading);
        double sin = Math.sin(-robotHeading);
        double robotX = fieldX * cos - fieldY * sin;
        double robotY = fieldX * sin + fieldY * cos;
        return new double[] { robotX, robotY };
    }

    // ========================= STATUS =========================

    /**
     * Check if we've reached the target and settled.
     */
    public boolean isAtTarget() {
        return isWithinTolerance() && settleTimer.milliseconds() >= SETTLE_TIME_MS;
    }

    /**
     * Check if currently within tolerance (but might not be settled yet).
     */
    public boolean isWithinTolerance() {
        double xError = Math.abs(targetX - currentX);
        double yError = Math.abs(targetY - currentY);
        double headingErrorDeg = Math.abs(Math.toDegrees(normalizeAngle(targetHeading - currentHeading)));

        return xError < POSITION_TOLERANCE &&
                yError < POSITION_TOLERANCE &&
                headingErrorDeg < HEADING_TOLERANCE;
    }

    /**
     * Stop all motors.
     */
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void resetPID() {
        xIntegral = 0;
        yIntegral = 0;
        headingIntegral = 0;
        xLastError = 0;
        yLastError = 0;
        headingLastError = 0;
        wasInTolerance = false;
        pidTimer.reset();
    }

    // ========================= UTILITY =========================

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // ========================= MANUAL CONTROL =========================

    /**
     * Manual drive for repositioning (bypasses PID).
     * @param forward Forward power (-1 to 1)
     * @param strafe Strafe power (-1 to 1, positive = right)
     * @param rotate Rotation power (-1 to 1, positive = clockwise)
     */
    public void manualDrive(double forward, double strafe, double rotate) {
        // Update pose even during manual control
        updatePose();

        double fl = forward + strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;

        // Normalize
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

    // ========================= TELEMETRY GETTERS =========================

    public double getCurrentX() { return currentX; }
    public double getCurrentY() { return currentY; }
    public double getCurrentHeadingDegrees() { return Math.toDegrees(currentHeading); }

    public double getTargetX() { return targetX; }
    public double getTargetY() { return targetY; }
    public double getTargetHeadingDegrees() { return Math.toDegrees(targetHeading); }

    public double getXError() { return targetX - currentX; }
    public double getYError() { return targetY - currentY; }
    public double getHeadingErrorDegrees() { return Math.toDegrees(normalizeAngle(targetHeading - currentHeading)); }
}