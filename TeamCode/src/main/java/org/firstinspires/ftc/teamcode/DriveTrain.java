package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class DriveTrain {
    HardwareMap hardwareMap;

    DcMotor frontLeftMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backRightMotor = null;

    Servo highSpeedCamera;

    Gamepad gamepad;

    private Integer lockedTagId = null;


    private static final double spin_speed = 0.3;
    private static final double headingTolerance = 1.0;
    private static final double maxTurnSpeed = 0.1;
    private static final double timeoutMs = 3000;

    Telemetry telemetry;

    private static final double tolerance = 0.1;

    UnifiedLocalization odo;

    public DriveTrain(HardwareMap hardwaremp, Gamepad gp, UnifiedLocalization odometry, Telemetry tmtry) {
        frontLeftMotor = hardwaremp.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwaremp.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwaremp.dcMotor.get("frontRightMotor");
        backRightMotor = hardwaremp.dcMotor.get("backRightMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        highSpeedCamera = hardwaremp.servo.get("cameraServo");

        gamepad = gp;

        odo = odometry;

        telemetry = tmtry;
    }
    double yvalue;
    double xvalue;

    public void drive() {
        // Shared power variables
        double y_power;
        double x_power;
        double rx_power;

        if (lockedTagId == null) {
            //  driver has full control.
            y_power = -gamepad.left_stick_y;
            x_power = gamepad.left_stick_x * 1.1; // Counteract imperfect strafing
            rx_power = gamepad.right_stick_x;
            highSpeedCamera.setPosition(0);
        } else {
            // mode 2, field centric with tag lock
            // driver controls translation robot controls rotation.

//            odo.updateAprilTag();
            AprilTagDetection lockedTag = odo.getDetectionById(lockedTagId);

            if (lockedTag != null) {
                double bearingDifference = lockedTag.ftcPose.bearing;
                double yawDifference = highSpeedCamera.getPosition() + ((lockedTag.ftcPose.pitch*57.2958)/360)/2; //normalized 0-1

                yvalue = yawDifference;
                xvalue = bearingDifference;


                rx_power = -Range.clip(bearingDifference, -maxTurnSpeed, maxTurnSpeed);


                highSpeedCamera.setPosition(Math.abs(yawDifference));

            } else {
                rx_power = 0;
            }


            //field y and x are driver,
            double field_y = -gamepad.left_stick_y;
            double field_x = gamepad.left_stick_x;

            // current heading in radians
            double headingRad = Math.toRadians(odo.getOdoHeading());

            // rotation logic but applied to gamepad inputs
            // rotates the field-centric stick inputs into robot-relative powers.
            x_power = field_x * Math.cos(-headingRad) - field_y * Math.sin(-headingRad);
            y_power = field_x * Math.sin(-headingRad) + field_y * Math.cos(-headingRad);


        }

        //same for both modes
        double denominator = Math.max(Math.abs(y_power) + Math.abs(x_power) + Math.abs(rx_power), 1);
        double frontLeftPower = (y_power + x_power + rx_power) / denominator;
        double backLeftPower = (y_power - x_power + rx_power) / denominator;
        double frontRightPower = (y_power - x_power - rx_power) / denominator;
        double backRightPower = (y_power + x_power - rx_power) / denominator;

        telemetry.addData("rx power", rx_power);
        telemetry.addData("lockedTagId", lockedTagId);
        telemetry.addData("servopos", highSpeedCamera.getPosition());
        telemetry.addData("centery", yvalue);
        telemetry.addData("centerx", xvalue);



        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    //Physics based drive to some coordinates coordinate
    //variables needed for speed:
    double maxRPM = 435; //max rpm of the motor
    double wheelDiameter = 4.09449; //wheel diameter in inches
    double topSpeed = (maxRPM / 60) * (Math.PI * wheelDiameter); //top speed in inches per second


    public void lockOntoTag(int tagId) {
        lockedTagId = tagId;
    }

    public void unlockFromTag() {
        this.lockedTagId = null;
    }



    public void goTo(double targetX, double targetY, double speed) {

        double currentX = odo.getOdoX();
        double currentY = odo.getOdoY();
        double currentHeading = odo.getOdoHeading();

        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;

        double distanceToTarget = Math.hypot(deltaX, deltaY); //didnt know this func existed till now

        while (Math.abs(distanceToTarget) > tolerance) {

            telemetry.addData("x y z", "%f, %f, %f", currentX, currentY, currentHeading);
            telemetry.update();
            // more rotation (spamming it now) (back to robot pov)
            double headingRad = Math.toRadians(currentHeading);
            double sinHeading = Math.sin(headingRad);
            double cosHeading = Math.cos(headingRad);

            // y = forward backwards to robot
            // x = strafe to robot
            double robotRelativeY = deltaY * cosHeading - deltaX * sinHeading;
            double robotRelativeX = deltaY * sinHeading + deltaX * cosHeading;

            //get direction by making the vector into a vector from components
            double moveMagnitude = Math.hypot(robotRelativeX, robotRelativeY);
            double powerX = (robotRelativeY / moveMagnitude) * speed;
            double powerY = -(robotRelativeX / moveMagnitude) * speed; //switched x and y here

            double powerRX = 0;

            double denominator = Math.max(Math.abs(powerY) + Math.abs(powerX) + Math.abs(powerRX), 1);
            double frontLeftPower = (powerY + powerX + powerRX) / denominator;
            double backLeftPower = (powerY - powerX + powerRX) / denominator;
            double frontRightPower = (powerY - powerX - powerRX) / denominator;
            double backRightPower = (powerY + powerX - powerRX) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            //update
            currentX = odo.getOdoX();
            currentY = odo.getOdoY();
            currentHeading = odo.getOdoHeading();
            deltaX = targetX - currentX;
            deltaY = targetY - currentY;
            distanceToTarget = Math.hypot(deltaX, deltaY);
        }
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public boolean findAndFaceTag(int targetId) {
        AprilTagDetection targetTag = null;
        ElapsedTime searchTimer = new ElapsedTime();
        searchTimer.reset();

        //search phase, just spin right

        frontLeftMotor.setPower(spin_speed);
        backLeftMotor.setPower(spin_speed);
        frontRightMotor.setPower(-spin_speed);
        backRightMotor.setPower(-spin_speed);

        // loop until the tag is found
        while (searchTimer.milliseconds() < timeoutMs) {
            odo.updateAprilTag();
            targetTag = odo.getDetectionById(targetId);

            if (targetTag != null) {
                break;
            }
        }
        //stop spinning
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        if (targetTag == null) {
            return false;
        }

        //alignment, use bearing to align
        while (Math.abs(targetTag.ftcPose.bearing) > headingTolerance) {
            // turning power, encorperating error and max turn speed
            double error = targetTag.ftcPose.bearing;
            double turnPower = Range.clip(error, -maxTurnSpeed, maxTurnSpeed);



            if (odo.getOdoHeading() - targetTag.ftcPose.bearing > 0) {
                rampSpin(0, turnPower, -0.01);

            } else if (odo.getOdoHeading() - targetTag.ftcPose.bearing < 0) {
                rampSpin(0, turnPower, 0.01);

            }

            odo.updateAprilTag();
            targetTag = odo.getDetectionById(targetId);

            // if lose the lock, return false
            if (targetTag == null) {
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                return false;
            }
        }

        // stop when aligned
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        return true;
    }

    public void rampSpin(double startingPower, double targetPower, double step) {
        for (double p = startingPower; p < targetPower; p += step) {
            frontLeftMotor.setPower(p);
            backLeftMotor.setPower(p);
            frontRightMotor.setPower(-p);
            backRightMotor.setPower(-p);
            try {
                Thread.sleep(50); // small delay to allow power change to take effect
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

        }
    }


    public void rampToXY(double targetX, double targetY, double tSpeed) {
        double startX = odo.getOdoX();
        double startY = odo.getOdoY();

        double deltaX = targetX - startX;
        double deltaY = targetY - startY;

        double totalDistance = Math.hypot(deltaX, deltaY);
        double toleranceInches = 0.1;

        double prevPower = 0;

        double remainingDistance = totalDistance;


        while (Math.abs(remainingDistance) >= toleranceInches) {
            double currentX = odo.getOdoX();
            double currentY = odo.getOdoY();

            double remainingX = targetX - currentX;
            double remainingY = targetY - currentY;
            remainingDistance = Math.hypot(remainingX, remainingY);

            // 0=start, 1=end, a percentage of how far we need to go
            double progress = 1.0 - (remainingDistance / totalDistance);

            // bell curve scaling factor
            double rampFactor = 4 * progress * (1 - progress); // parabola
//            rampFactor = Math.min(Math.max(rampFactor, 0.05), 1.0); // avoid tiny powers so it doesnt go crazy

            // normalize ts
            double targetPower = rampFactor * (tSpeed / topSpeed);

            // movement calcs
            double moveAngle = Math.atan2(remainingY, remainingX);
            double powerY = -(Math.cos(moveAngle) * targetPower);
            double powerX = -(Math.sin(moveAngle) * targetPower);

            // same as drive()ish
            double denominator = Math.max(Math.abs(powerY) + Math.abs(powerX), 1);
            double frontLeftPower = (powerY + powerX) / denominator;
            double backLeftPower = (powerY - powerX) / denominator;
            double frontRightPower = (powerY - powerX) / denominator;
            double backRightPower = (powerY + powerX) / denominator;

            // integrate from previous power so its smooth
            double maxMotorPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(backLeftPower),
                    Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));

            if (maxMotorPower > 0) {
                double scale = targetPower / maxMotorPower;
                frontLeftPower *= scale;
                backLeftPower *= scale;
                frontRightPower *= scale;
                backRightPower *= scale;
            }

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

}

