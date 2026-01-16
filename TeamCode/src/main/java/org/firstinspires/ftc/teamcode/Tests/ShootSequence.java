//package org.firstinspires.ftc.teamcode.Tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Robot;
//
//@TeleOp(name = "flickyy", group = "Testing")
//public class ShootSequence extends OpMode {
//
//    private Robot r;
//
//    private enum BallColor { GREEN, PURPLE, EMPTY }
//
//    //ball config
//    private BallColor[] slots = { BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE };
//
//    // target sequence
//    private BallColor[] targetSequence = { BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE };
//
//    // seq stuff
//    private int sequenceIndex = 0;
//    private static final int SHOOTING_POSITION = 0;
//
//    private enum State { MOVING, FLICKING, DONE }
//    private State state = State.MOVING;
//
//    // Carousel
//    private static final double POWER = 0.1;
//    private static final int NUM_POSITIONS = 3;
//
//
//    // Hole tracking
//    private boolean wasInHole = false;
//    private int holeCount = 0;
//
//    // Position tracking
//    private int currentPosition = 0;
//    private int targetPosition = 0;
//    private int direction = 0;
//
//    // Flicker
//    private static final double DOWN = 0;
//    private static final double UP = 0.4;
//    private static final double FLICK_DURATION = 1.0;
//    private ElapsedTime flickTimer = new ElapsedTime();
//
//    @Override
//    public void init() {
//        r = new Robot(hardwareMap);
//        r.init();
//
//        initHoleTracking();
//        queueNextShot();
//    }
//
//    @Override
//    public void loop() {
//        trackHoles();
//
//        switch (state) {
//            case MOVING:
//                updateCarousel();
//                break;
//            case FLICKING:
//                updateFlicker();
//                break;
//            case DONE:
//                r.carouselMotor.setPower(0);
//                break;
//        }
//
//        updateTelemetry();
//    }
//
//    // Init methods
//
//
//    private void initHoleTracking() {
//        wasInHole = isFinInHole();
//        holeCount = 0;
//    }
//
//    // Fin/hole methods
//
//    private void trackHoles() {
//        boolean inHole = isFinInHole();
//
//        if (inHole && !wasInHole) {
//            holeCount++;
//
//            if (isAlignmentHole()) {
//                currentPosition = (currentPosition + direction + NUM_POSITIONS) % NUM_POSITIONS;
//            }
//        }
//
//        wasInHole = inHole;
//    }
//
//    private boolean isFinInHole() {
//        return !r.leftLim.getState() || !r.rightLim.getState();
//    }
//
//    private boolean isAlignmentHole() {
//        return holeCount % 2 == 0;
//    }
//
//    private boolean isInAlignmentHole() {
//        return isFinInHole() && isAlignmentHole();
//    }
//
//    // Sequence methods
//
//    private void queueNextShot() {
//        if (sequenceIndex >= targetSequence.length) {
//            state = State.DONE;
//            return;
//        }
//
//        BallColor nextColor = targetSequence[sequenceIndex];
//        int slotWithBall = findSlotWithColor(nextColor);
//
//        if (slotWithBall == -1) {
//            sequenceIndex++;
//            queueNextShot();
//            return;
//        }
//
//        setTargetPosition(slotWithBall);
//        state = State.MOVING;
//    }
//
//    private int findSlotWithColor(BallColor color) {
//        for (int i = 0; i < slots.length; i++) {
//            if (slots[i] == color) {
//                return i;
//            }
//        }
//        return -1;
//    }
//
//    private void shootBall() {
//        slots[currentPosition] = BallColor.EMPTY;
//        sequenceIndex++;
//    }
//
//    // Carousel methods
//
//    private void setTargetPosition(int newTarget) {
//        targetPosition = newTarget;
//        direction = calculateDirection();
//        holeCount = 0;
//        wasInHole = isFinInHole();
//    }
//
//    private int calculateDirection() {
//        if (currentPosition == targetPosition) {
//            return 0;
//        }
//
//        int forwardDist = (targetPosition - currentPosition + NUM_POSITIONS) % NUM_POSITIONS;
//        int backwardDist = (currentPosition - targetPosition + NUM_POSITIONS) % NUM_POSITIONS;
//
//        return (forwardDist <= backwardDist) ? 1 : -1;
//    }
//
//    private void updateCarousel() {
//        if (isCarouselSettled()) {
//            r.carouselMotor.setPower(0);
//            direction = 0;
//            state = State.FLICKING;
//            flickTimer.reset();
//        } else {
//            r.carouselMotor.setPower(POWER * direction);
//        }
//    }
//
//    private boolean isCarouselSettled() {
//        return currentPosition == targetPosition && isInAlignmentHole();
//    }
//
//    // Flicker methods
//
//    private void updateFlicker() {
//        if (flickTimer.seconds() < FLICK_DURATION) {
//            r.kicker.setPosition(UP);
//        } else {
//            r.kicker.setPosition(DOWN);
//            shootBall();
//            queueNextShot();
//        }
//    }
//
//    // Telemetry
//
//    private void updateTelemetry() {
//        telemetry.addData("State", state);
//        telemetry.addData("Progress", sequenceIndex + "/" + targetSequence.length);
//
//        telemetry.addData("Slot 1", slots[0]);
//        telemetry.addData("Slot 2", slots[1]);
//        telemetry.addData("Slot 3", slots[2]);
//
//        telemetry.addData("Current Pos", currentPosition);
//        telemetry.addData("Target Pos", targetPosition);
//
//        telemetry.update();
//    }
//}