package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "flickyy", group = "Testing")
public class FullFlick extends OpMode {

    private enum BallColor { GREEN, PURPLE, EMPTY }

    //ball config
    private BallColor[] slots = { BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE };

    // target sequence
    private BallColor[] targetSequence = { BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE };

    // seq stuff
    private int sequenceIndex = 0;
    private static final int SHOOTING_POSITION = 0;

    private enum State { MOVING, FLICKING, DONE }
    private State state = State.MOVING;

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
    private int direction = 0;

    // Flicker
    private Servo flickerServo;
    private static final double DOWN = 0;
    private static final double UP = 0.4;
    private static final double FLICK_DURATION = 1.0;
    private ElapsedTime flickTimer = new ElapsedTime();

    @Override
    public void init() {
        initCarousel();
        initLimitSwitches();
        initHoleTracking();
        initFlicker();
        queueNextShot();
    }

    @Override
    public void loop() {
        trackHoles();

        switch (state) {
            case MOVING:
                updateCarousel();
                break;
            case FLICKING:
                updateFlicker();
                break;
            case DONE:
                carouselMotor.setPower(0);
                break;
        }

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

    // Sequence methods

    private void queueNextShot() {
        if (sequenceIndex >= targetSequence.length) {
            state = State.DONE;
            return;
        }

        BallColor nextColor = targetSequence[sequenceIndex];
        int slotWithBall = findSlotWithColor(nextColor);

        if (slotWithBall == -1) {
            sequenceIndex++;
            queueNextShot();
            return;
        }

        setTargetPosition(slotWithBall);
        state = State.MOVING;
    }

    private int findSlotWithColor(BallColor color) {
        for (int i = 0; i < slots.length; i++) {
            if (slots[i] == color) {
                return i;
            }
        }
        return -1;
    }

    private void shootBall() {
        slots[currentPosition] = BallColor.EMPTY;
        sequenceIndex++;
    }

    // Carousel methods

    private void setTargetPosition(int newTarget) {
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
            state = State.FLICKING;
            flickTimer.reset();
        } else {
            carouselMotor.setPower(POWER * direction);
        }
    }

    private boolean isCarouselSettled() {
        return currentPosition == targetPosition && isInAlignmentHole();
    }

    // Flicker methods

    private void updateFlicker() {
        if (flickTimer.seconds() < FLICK_DURATION) {
            flickerServo.setPosition(UP);
        } else {
            flickerServo.setPosition(DOWN);
            shootBall();
            queueNextShot();
        }
    }

    // Telemetry

    private void updateTelemetry() {
        telemetry.addData("State", state);
        telemetry.addData("Progress", sequenceIndex + "/" + targetSequence.length);

        telemetry.addData("Slot 1", slots[0]);
        telemetry.addData("Slot 2", slots[1]);
        telemetry.addData("Slot 3", slots[2]);

        telemetry.addData("Current Pos", currentPosition);
        telemetry.addData("Target Pos", targetPosition);

        telemetry.update();
    }
}