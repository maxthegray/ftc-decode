package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class DriveTrain {
    HardwareMap hardwareMap;

    DcMotor frontLeftMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backRightMotor = null;

    Gamepad gamepad;

    private static final double spin_speed = 0.3;
    private static final double headingTolerance = 1.0;
    private static final double maxTurnSpeed = 0.1;
    private static final double timeoutMs = 3000;


    private static final double tolerance = 0.5;

    UnifiedLocalization odo;

    public DriveTrain(HardwareMap hardwaremp, Gamepad gp, UnifiedLocalization odometry) {
        frontLeftMotor = hardwaremp.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwaremp.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwaremp.dcMotor.get("frontRightMotor");
        backRightMotor = hardwaremp.dcMotor.get("backRightMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        gamepad = gp;

        odo = odometry;
    }

    public void drive() {

        double y = -gamepad.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad.left_stick_x * 1; // Counteract imperfect strafing
        double rx = gamepad.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    //Physics based drive to some coordinates coordinate
    //variables needed for speed:
    double maxRPM; //max rpm of the motor
    double wheelDiameter; //wheel diameter in inches
    double topSpeed = (maxRPM / 60) * (Math.PI * wheelDiameter); //top speed in inches per second



    public void goTo(double targetX, double targetY, double speed) {

        double currentX = odo.getOdoX();
        double currentY = odo.getOdoY();
        double currentHeading = odo.getOdoHeading();

        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;

        double distanceToTarget = Math.hypot(deltaX, deltaY); //didnt know this func existed till now

        while (distanceToTarget > tolerance) {
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
            double powerY = (robotRelativeY / moveMagnitude) * speed;
            double powerX = (robotRelativeX / moveMagnitude) * speed;

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
                ramp(-1/2, 0, turnPower, 0.01);

            } else if (odo.getOdoHeading() - targetTag.ftcPose.bearing < 0) {
                ramp(1/2, 0, turnPower, 0.01);

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

    public void ramp(int direction, double currentPower, double targetPower, double step) {
        if (direction == 1) { // ramping up
            for (double p = currentPower; p < targetPower; p += step) {
                frontLeftMotor.setPower(p);
                backLeftMotor.setPower(p);
                frontRightMotor.setPower(p);
                backRightMotor.setPower(p);
                try {
                    Thread.sleep(50); // small delay to allow power change to take effect
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }else if (direction == -1) { // ramping down
                for (double p = currentPower; p > targetPower; p -= step) {
                    frontLeftMotor.setPower(p);
                    backLeftMotor.setPower(p);
                    frontRightMotor.setPower(p);
                    backRightMotor.setPower(p);
                    try {
                        Thread.sleep(50); // small delay to allow power change to take effect
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }

            } else if (direction == 1/2) {
                for (double p = currentPower; p < targetPower; p += step) {
                    frontLeftMotor.setPower(p);
                    backLeftMotor.setPower(p);
                    frontRightMotor.setPower(-p);
                    backRightMotor.setPower(-p);
                    try {
                        Thread.sleep(50); // small delay to let power change do the thing
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }
            } else if (direction == -1/2) {
                for (double p = currentPower; p > targetPower; p -= step) {
                    frontLeftMotor.setPower(p);
                    backLeftMotor.setPower(p);
                    frontRightMotor.setPower(-p);
                    backRightMotor.setPower(-p);
                    try {
                        Thread.sleep(50);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }
            }


        }
    }
}
