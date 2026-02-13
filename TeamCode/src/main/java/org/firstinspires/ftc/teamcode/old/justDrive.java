package org.firstinspires.ftc.teamcode.old;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled
@TeleOp(name = "Drive + Carousel (Pedro)", group = "TeleOp")
public class justDrive extends OpMode {

    // Drive
    private Follower follower;

    // Carousel
    private int targetPosition = 0;
    private static final int TICKS_PER_REV = 2234;
    private static final int POSITION_1 = 0;
    private static final int POSITION_2 = TICKS_PER_REV / 3;
    private static final int POSITION_3 = (TICKS_PER_REV / 3) * 2;
    private static final double CAROUSEL_POWER = 0.4;
    private static final int POSITION_TOLERANCE = 5;

    // Flicker
    private double flickerPosition = 0;
    private static final double FLICKER_DOWN = 0;
    private static final double FLICKER_UP = 0.4;

    private boolean intakeOn = false;

    private Robot r;


    @Override
    public void init() {
        r = new Robot(hardwareMap);
        r.init();

        // Initialize drive
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.startTeleOpDrive();
        follower.setTeleOpDrive(0,0,0);

    }

    @Override
    public void loop() {
        follower.update();

        // Drive
        handleDrive();


        // Carousel
        handleCarouselInput();
        updateTelemetry();

        updateCarousel();

        // Flicker
        handleFlickerInput();
        updateFlicker();

        if (gamepad2.right_bumper) {
            r.Intake1.setPower(.75);
            r.Intake2.setPower(.75);
        }
        if (gamepad2.left_bumper) {
            r.Intake1.setPower(0);
            r.Intake2.setPower(0);
        }

        // Telemetry
        updateTelemetry();
    }

    // Drive

    private void handleDrive() {
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotation = -gamepad1.right_stick_x;

        follower.setTeleOpDrive(forward, strafe, rotation);
        follower.update();
    }

    // Carousel

    private void handleCarouselInput() {
        if (gamepad2.square) {
            targetPosition = POSITION_1;
        } else if (gamepad2.triangle) {
            targetPosition = POSITION_2;
        } else if (gamepad2.circle) {
            targetPosition = POSITION_3;
        }
    }

    private void updateCarousel() {
        r.carouselMotor.setTargetPosition(targetPosition);
        r.carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        r.carouselMotor.setPower(CAROUSEL_POWER);
    }

    private boolean isCarouselSettled() {
        int error = Math.abs(r.carouselMotor.getCurrentPosition() - targetPosition);
        return error <= POSITION_TOLERANCE;
    }

    // Flicker

    private void handleFlickerInput() {
        if (!isCarouselSettled()) {
            return;
        }
        if (gamepad2.dpad_up) {
            flickerPosition = FLICKER_UP;
        } else if (gamepad2.dpad_down) {
            flickerPosition = FLICKER_DOWN;
        }
    }

    private void updateFlicker() {
        r.kicker.setPosition(flickerPosition);
    }

    private void handleIntake() {
        if (gamepad2.right_bumper) {
            r.setIntakeMotors(-0.75);
        }
        else if (gamepad2.left_bumper) {
            r.setIntakeMotors(0.75);
        }
        else if (gamepad2.left_trigger > 0) {
            r.setIntakeMotors(0);
        }
    }

    // Telemtry

    private void updateTelemetry() {
        //drive info
        Pose pose = follower.getPose();
        telemetry.addData("stuff", gamepad1.right_stick_y);
        telemetry.addData("--- DRIVE ---", "");
        telemetry.addData("X", "%.2f inches", pose.getX());
        telemetry.addData("Y", "%.2f inches", pose.getY());
        telemetry.addData("Heading", "%.2f degrees", Math.toDegrees(pose.getHeading()));

        // Carousel info
        telemetry.addLine();
        telemetry.addData("--- CAROUSEL ---", "");
        telemetry.addData("Target", targetPosition);
        telemetry.addData("Current", r.carouselMotor.getCurrentPosition());
        telemetry.addData("Power", "%.2f", r.carouselMotor.getPower());
        telemetry.addData("Settled", isCarouselSettled() ? "YES" : "NO");

        // Flicker info
        telemetry.addLine();
        telemetry.addData("--- FLICKER ---", "");
        telemetry.addData("Position", "%.2f", flickerPosition);
        telemetry.addData("State", flickerPosition == FLICKER_UP ? "UP" : "DOWN");

        telemetry.update();
    }
}