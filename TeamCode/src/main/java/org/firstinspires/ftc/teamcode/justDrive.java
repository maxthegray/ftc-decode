package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Drive + Carousel (Pedro)", group = "TeleOp")
public class justDrive extends OpMode {

    // Drive
    private Follower follower;

    // Carousel
    private DcMotor carouselMotor;
    private int targetPosition = 0;
    private static final int TICKS_PER_REV = 6700 / 4;
    private static final int POSITION_1 = 0;
    private static final int POSITION_2 = TICKS_PER_REV / 3;
    private static final int POSITION_3 = (TICKS_PER_REV / 3) * 2;
    private static final double CAROUSEL_POWER = 0.6;
    private static final int POSITION_TOLERANCE = 5;

    // Flicker
    private Servo flickerServo;
    private double flickerPosition = 0;
    private static final double FLICKER_DOWN = 0;
    private static final double FLICKER_UP = 0.4;

    // Limit switches
    private DigitalChannel leftFinLimit;
    private DigitalChannel rightFinLimit;

    @Override
    public void init() {
        // Initialize drive
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.startTeleOpDrive();
        follower.setTeleOpDrive(0,0,0);

        // Initialize carousel
        carouselMotor = hardwareMap.dcMotor.get("carouselMotor");
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize flicker
        flickerServo = hardwareMap.get(Servo.class, "flickServo");
        flickerPosition = FLICKER_DOWN;

        // Initialize limit switches
        leftFinLimit = hardwareMap.get(DigitalChannel.class, "leftFin");
        leftFinLimit.setMode(DigitalChannel.Mode.INPUT);

        rightFinLimit = hardwareMap.get(DigitalChannel.class, "rightFin");
        rightFinLimit.setMode(DigitalChannel.Mode.INPUT);
        follower.update();
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

        // Telemetry
        updateTelemetry();
    }

    // ==================== DRIVE ====================

    private void handleDrive() {
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotation = -gamepad1.right_stick_x;

        follower.setTeleOpDrive(forward, strafe, rotation);
        follower.update();
    }

    // ==================== CAROUSEL ====================

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
        carouselMotor.setTargetPosition(targetPosition);
        carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carouselMotor.setPower(CAROUSEL_POWER);
    }

    private boolean isCarouselSettled() {
        int error = Math.abs(carouselMotor.getCurrentPosition() - targetPosition);
        return error <= POSITION_TOLERANCE;
    }

    // ==================== FLICKER ====================

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
        flickerServo.setPosition(flickerPosition);
    }

    // ==================== TELEMETRY ====================

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
        telemetry.addData("Current", carouselMotor.getCurrentPosition());
        telemetry.addData("Power", "%.2f", carouselMotor.getPower());
        telemetry.addData("Settled", isCarouselSettled() ? "YES" : "NO");

        // Flicker info
        telemetry.addLine();
        telemetry.addData("--- FLICKER ---", "");
        telemetry.addData("Position", "%.2f", flickerPosition);
        telemetry.addData("State", flickerPosition == FLICKER_UP ? "UP" : "DOWN");

        // Limit switches
        telemetry.addLine();
        telemetry.addData("--- LIMIT SWITCHES ---", "");
        telemetry.addData("Left Fin", leftFinLimit.getState() ? "P" : "NP");
        telemetry.addData("Right Fin", rightFinLimit.getState() ? "P" : "NP");

        telemetry.update();
    }
}