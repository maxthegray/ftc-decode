package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Localization.UnifiedLocalization;
import org.firstinspires.ftc.teamcode.Shooter.ShooterCamera;

public class DriveTrain {
    HardwareMap hardwareMap;

    DcMotor frontLeftMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backRightMotor = null;

    Gamepad gamepad;

    ShooterCamera shooterCamera;
    UnifiedLocalization Location;

    private boolean locked = false;


    private static final double spin_speed = 0.3;
    private static final double headingTolerance = 1.0;
    private static final double maxTurnSpeed = 0.1;
    private static final double timeoutMs = 3000;

    Telemetry telemetry;

    private static final double tolerance = 0.1;



    public DriveTrain(HardwareMap hardwaremp, Gamepad gp, Telemetry tmtry) {
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

        shooterCamera = new ShooterCamera(tmtry, hardwaremp);

        Location = new UnifiedLocalization(tmtry, hardwaremp);

        gamepad = gp;


        telemetry = tmtry;
    }
    double yvalue;
    double xvalue;

    public void drive() {
        // Shared power variables
        double y_power;
        double x_power;
        double rx_power;

        if (!locked) {
            //  driver has full control.
            y_power = -gamepad.left_stick_y;
            x_power = gamepad.left_stick_x * 1; // Counteract imperfect strafing
            rx_power = gamepad.right_stick_x;

        } else {
            shooterCamera.alignCameraToTag();
            rx_power = shooterCamera.alignRobotToTagPower();

            // current heading in radians
//            double headingRad = Math.toRadians(odo.getOdoHeading());

            // rotation logic but applied to gamepad inputs
            // rotates the field-centric stick inputs into robot-relative powers.
//            double field_y = -gamepad.left_stick_y;
//            double field_x = gamepad.left_stick_x;
//            x_power = field_x * Math.cos(-headingRad) - field_y * Math.sin(-headingRad);
//            y_power = field_x * Math.sin(-headingRad) + field_y * Math.cos(-headingRad);
            y_power = -gamepad.left_stick_y;
            x_power = gamepad.left_stick_x * 1; // Counteract imperfect strafing
        }

        //same for both modes
        double denominator = Math.max(Math.abs(y_power) + Math.abs(x_power) + Math.abs(rx_power), 1);
        double frontLeftPower = (y_power + x_power + rx_power) / denominator;
        double backLeftPower = (y_power - x_power + rx_power) / denominator;
        double frontRightPower = (y_power - x_power - rx_power) / denominator;
        double backRightPower = (y_power + x_power - rx_power) / denominator;

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


    public void lockOntoTag() {
        locked = true;
    }

    public void unlockFromTag() {
        locked = false;
    }



    public void goTo(double targetX, double targetY, double speed) {

        double currentX = Location.getX();
        double currentY = Location.getY();
        double currentHeading = Location.getHeading();

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
            currentX = Location.getX();
            currentY = Location.getY();
            currentHeading = Location.getHeading();
            deltaX = targetX - currentX;
            deltaY = targetY - currentY;
            distanceToTarget = Math.hypot(deltaX, deltaY);
        }
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public void rampToXYH(double targetX, double targetY, double targetHeading, double tSpeed) {
        double startX = Location.getX();
        double startY = Location.getY();
        double startHeading = Location.getHeading();

        double deltaX = targetX - startX;
        double deltaY = targetY - startY;

        double totalDistance = Math.hypot(deltaX, deltaY);
        double toleranceInches = 0.1;
        double toleranceHeading = 2.0;

        double remainingDistance = totalDistance;
        double remainingHeading = angleSimplifierDeg(targetHeading - startHeading);

        while (Math.abs(remainingDistance) >= toleranceInches || Math.abs(remainingHeading) >= toleranceHeading) {
            double currentX = Location.getX();
            double currentY = Location.getY();
            double currentHeading = Location.getHeading();

            double remainingX = targetX - currentX;
            double remainingY = targetY - currentY;
            remainingDistance = Math.hypot(remainingX, remainingY);

            remainingHeading = angleSimplifierDeg(targetHeading - currentHeading);

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

            double kRot = 0.02;
            double turnPower = kRot * remainingHeading;

            // same as drive()ish
            double denominator = Math.max(Math.abs(powerY) + Math.abs(powerX) + Math.abs(turnPower), 1);
            double frontLeftPower = (powerY + powerX + turnPower) / denominator;
            double backLeftPower = (powerY - powerX + turnPower) / denominator;
            double frontRightPower = (powerY - powerX - turnPower) / denominator;
            double backRightPower = (powerY + powerX - turnPower) / denominator;

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

    private double angleSimplifierDeg(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}