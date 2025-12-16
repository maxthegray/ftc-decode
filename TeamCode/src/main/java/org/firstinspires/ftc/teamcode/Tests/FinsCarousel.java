package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "CarouselTest", group = "Testing")
public class FinsCarousel extends OpMode {

    // Carousel
    private DcMotor carouselMotor;
    private static final double POWER = 0.4;
    private static final int NUM_POSITIONS = 3;

    // Limit switches (fins)
    private DigitalChannel leftFinLimit;
    private DigitalChannel rightFinLimit;

    // Hole tracking
    private boolean wasInHole = false;
    private int holeCount = 0;

    // Position tracking
    private int currentPosition = 0;
    private int targetPosition = 0;
    private int direction = 0; // 1 = forward, -1 = backward, 0 = stopped

    // Flicker
    private Servo flickerServo;
    private static final double DOWN = 0;
    private static final double UP = 0.4;
    private static final double FLICK_DURATION = 1.0;
    private ElapsedTime flickTimer = new ElapsedTime();
    private boolean isFlicking = false;

    @Override
    public void init() {
        initCarousel();
        initLimitSwitches();
        initHoleTracking();
        initFlicker();
    }

    @Override
    public void loop() {
        trackHoles();
        handleCarouselInput();
        updateCarousel();
        handleFlickerInput();
        updateFlicker();
        updateTelemetry();
    }

    // Init methods

    private void initCarousel() {
        carouselMotor = hardwareMap.dcMotor.get("carouselMotor");
        carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initLimitSwitches() {
        leftFinLimit = hardwareMap.get(DigitalChannel.class, "leftFin");
        leftFinLimit.setMode(DigitalChannel.Mode.INPUT);

        rightFinLimit = hardwareMap.get(DigitalChannel.class, "rightFin");
        rightFinLimit.setMode(DigitalChannel.Mode.INPUT);
    }

    private void initHoleTracking() {
        wasInHole = isFinInHole();
        holeCount = 0;
    }

    private void initFlicker() {
        flickerServo = hardwareMap.get(Servo.class, "flickServo");
        flickerServo.setPosition(DOWN);
    }

    // Fin/hole methods

    private void trackHoles() {
        boolean inHole = isFinInHole();

        if (inHole && !wasInHole) {
            holeCount++;

            if (isAlignmentHole()) {
                currentPosition = (currentPosition + direction + NUM_POSITIONS) % NUM_POSITIONS;
            }
        }

        wasInHole = inHole;
    }

    private boolean isFinInHole() {
        return leftFinLimit.getState() || rightFinLimit.getState();
    }

    private boolean isAlignmentHole() {
        return holeCount % 2 == 0;
    }

    private boolean isInAlignmentHole() {
        return isFinInHole() && isAlignmentHole();
    }

    // Carousel methods

    private void handleCarouselInput() {
        if (gamepad1.square) {
            setTargetPosition(0);
        } else if (gamepad1.triangle) {
            setTargetPosition(1);
        } else if (gamepad1.circle) {
            setTargetPosition(2);
        }
    }

    private void setTargetPosition(int newTarget) {
        if (newTarget == targetPosition) {
            return;
        }

        targetPosition = newTarget;
        direction = calculateDirection();
        holeCount = 0;
        wasInHole = isFinInHole();
    }

    private int calculateDirection() {
        if (currentPosition == targetPosition) {
            return 0;
        }

        int forwardDist = (targetPosition - currentPosition + NUM_POSITIONS) % NUM_POSITIONS;
        int backwardDist = (currentPosition - targetPosition + NUM_POSITIONS) % NUM_POSITIONS;

        return (forwardDist <= backwardDist) ? 1 : -1;
    }

    private void updateCarousel() {
        if (isCarouselSettled()) {
            carouselMotor.setPower(0);
            direction = 0;
        } else {
            carouselMotor.setPower(POWER * direction);
        }
    }

    private boolean isCarouselSettled() {
        return currentPosition == targetPosition && isInAlignmentHole();
    }

    // Flicker methods

    private void handleFlickerInput() {
        if (!isCarouselSettled() || isFlicking) {
            return;
        }

        if (gamepad1.dpad_up) {
            isFlicking = true;
            flickTimer.reset();
        }
    }

    private void updateFlicker() {
        if (isFlicking) {
            if (flickTimer.seconds() < FLICK_DURATION) {
                flickerServo.setPosition(UP);
            } else {
                flickerServo.setPosition(DOWN);
                isFlicking = false;
            }
        } else {
            flickerServo.setPosition(DOWN);
        }
    }

    // Telemetry

    private void updateTelemetry() {
        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Direction", direction);
        telemetry.addData("Settled", isCarouselSettled() ? "Y" : "N");

        telemetry.addData("In Hole", isFinInHole() ? "Y" : "N");
        telemetry.addData("Hole Count", holeCount);

        telemetry.addData("Flicking", isFlicking ? "Y" : "N");

        telemetry.update();
    }
}