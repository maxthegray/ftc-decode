// ============================================
// CarouselSubsystem.java
// ============================================

package org.firstinspires.ftc.teamcode.commandbased.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CarouselSubsystem extends SubsystemBase {

    public enum BallColor { GREEN, PURPLE, EMPTY }

    // Hardware
    private final DcMotor carouselMotor;
    private final DigitalChannel leftFin;
    private final DigitalChannel rightFin;
    private final Servo kickerServo;

    // Constants
    private static final double CAROUSEL_POWER = 0.4;
    private static final int NUM_POSITIONS = 3;
    private static final double KICKER_DOWN = 0.0;
    private static final double KICKER_UP = 0.4;

    // Position tracking
    private int currentPosition = 0;
    private int targetPosition = 0;
    private int direction = 0;

    // Hole tracking
    private boolean wasInHole = false;
    private int holeCount = 0;

    // Slot contents
    private BallColor[] slots = { BallColor.EMPTY, BallColor.EMPTY, BallColor.EMPTY };

    public CarouselSubsystem(HardwareMap hardwareMap) {
        carouselMotor = hardwareMap.dcMotor.get("carousel_motor");
        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFin = hardwareMap.get(DigitalChannel.class, "leftFin");
        leftFin.setMode(DigitalChannel.Mode.INPUT);

        rightFin = hardwareMap.get(DigitalChannel.class, "rightFin");
        rightFin.setMode(DigitalChannel.Mode.INPUT);

        kickerServo = hardwareMap.get(Servo.class, "flicker_servo");
        kickerServo.setPosition(KICKER_DOWN);

        // Initialize hole tracking
        wasInHole = isFinInHole();
    }

    @Override
    public void periodic() {
        trackHoles();
        updateMotor();
    }



    public void goToPosition(int position) {
        if (position < 0 || position >= NUM_POSITIONS) {
            return;
        }

        targetPosition = position;
        direction = calculateDirection();
        holeCount = 0;
        wasInHole = isFinInHole();
    }

    public int getCurrentPosition() {
        return currentPosition;
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public boolean isSettled() {
        return currentPosition == targetPosition && isInAlignmentHole();
    }

    public void stop() {
        targetPosition = currentPosition;
        direction = 0;
        carouselMotor.setPower(0);
    }



    public void setKickerUp() {
        kickerServo.setPosition(KICKER_UP);
    }

    public void setKickerDown() {
        kickerServo.setPosition(KICKER_DOWN);
    }



    public BallColor getSlotContents(int slot) {
        if (slot < 0 || slot >= NUM_POSITIONS) {
            return BallColor.EMPTY;
        }
        return slots[slot];
    }

    public void setSlotContents(int slot, BallColor color) {
        if (slot >= 0 && slot < NUM_POSITIONS) {
            slots[slot] = color;
        }
    }

    public void setSlotEmpty(int slot) {
        setSlotContents(slot, BallColor.EMPTY);
    }

    public int findSlotWithColor(BallColor color) {
        for (int i = 0; i < slots.length; i++) {
            if (slots[i] == color) {
                return i;
            }
        }
        return -1;
    }

    public void setAllSlots(BallColor slot0, BallColor slot1, BallColor slot2) {
        slots[0] = slot0;
        slots[1] = slot1;
        slots[2] = slot2;
    }




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

    private void updateMotor() {
        if (isSettled()) {
            carouselMotor.setPower(0);
            direction = 0;
        } else {
            carouselMotor.setPower(CAROUSEL_POWER * direction);
        }
    }

    private int calculateDirection() {
        if (currentPosition == targetPosition) {
            return 0;
        }

        int forwardDist = (targetPosition - currentPosition + NUM_POSITIONS) % NUM_POSITIONS;
        int backwardDist = (currentPosition - targetPosition + NUM_POSITIONS) % NUM_POSITIONS;

        return (forwardDist <= backwardDist) ? 1 : -1;
    }

    private boolean isFinInHole() {
        return leftFin.getState() || rightFin.getState();
    }

    private boolean isAlignmentHole() {
        return holeCount % 2 == 0;
    }

    private boolean isInAlignmentHole() {
        return isFinInHole() && isAlignmentHole();
    }

// telemetry stuff
    public int getDirection() {
        return direction;
    }

    public int getHoleCount() {
        return holeCount;
    }

    public boolean getLeftFinState() {
        return leftFin.getState();
    }

    public boolean getRightFinState() {
        return rightFin.getState();
    }
}