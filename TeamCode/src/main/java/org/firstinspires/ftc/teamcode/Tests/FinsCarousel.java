package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Carousel fins localization", group = "Testing")
public class FinsCarousel extends OpMode {


    private Robot r;
    // Carousel
    private static final double POWER = 0.1;
    private static final int NUM_POSITIONS = 3;



    // Hole tracking
    private boolean wasInHole = false;
    private int holeCount = 1;

    // Position tracking
    private int currentPosition = 0;
    private int targetPosition = 0;
    private int direction = 0; // 1 = forward, -1 = backward, 0 = stopped

    // Flicker
    private static final double DOWN = 0;
    private static final double UP = 0.4;
    private static final double FLICK_DURATION = 0.2;
    private ElapsedTime flickTimer = new ElapsedTime();
    private boolean isFlicking = false;

    @Override
    public void init() {
        r = new Robot(hardwareMap);
        r.init();
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
        return !r.leftLim.getState() || !r.rightLim.getState();
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
        if (isCarouselSettled() && targetPosition == currentPosition) {
            r.carouselMotor.setPower(0);
            direction = 0;
        } else {
            r.carouselMotor.setPower(POWER * direction);
        }
    }

    private boolean isCarouselSettled() {
        return currentPosition == targetPosition && isInAlignmentHole();
//        return true;
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
                r.kicker.setPosition(UP);
            } else {
                r.kicker.setPosition(DOWN);
                isFlicking = false;
            }
        } else {
            r.kicker.setPosition(DOWN);
        }
    }

    // Telemetry

    private void updateTelemetry() {
        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Direction", direction);
        telemetry.addData("Settled", isCarouselSettled() ? "Y" : "N");

        telemetry.addData("Left", r.getState(r.leftLim) ? "Y" : "N");
        telemetry.addData("Right", r.getState(r.rightLim) ? "Y" : "N");

        telemetry.addData("Hole Count", holeCount);

        telemetry.addData("Flicking", isFlicking ? "Y" : "N");

        telemetry.update();
    }
}