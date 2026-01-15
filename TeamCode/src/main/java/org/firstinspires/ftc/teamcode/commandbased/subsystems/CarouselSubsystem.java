package org.firstinspires.ftc.teamcode.commandbased.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CarouselSubsystem extends SubsystemBase {

    public enum BallColor { GREEN, PURPLE, EMPTY, UNKNOWN }

    public static final int POS_INTAKE = 0;
    public static final int POS_BACK_LEFT = 1;
    public static final int POS_BACK_RIGHT = 2;

    private final DcMotor carouselMotor;
    private final DcMotor leftIntake;
    private final DcMotor rightIntake;
    private final Servo kickerServo;

    private final Servo light1, light2, light3;

    private final RevColorSensorV3[] sensorsA = new RevColorSensorV3[3];
    private final RevColorSensorV3[] sensorsB = new RevColorSensorV3[3];

    private static final double LIGHT_OFF = 0.0;
    private static final double LIGHT_GREEN = 0.555;
    private static final double LIGHT_PURPLE = 0.722;

    private static double CAROUSEL_POWER = 0.6;
    private static final double INTAKE_POWER = 0.75;
    private static final double KICKER_DOWN = 0.0;
    private static final double KICKER_UP = 0.4;

    public static int TICKS_PER_ROTATION = 2320;
    public static int TICKS_PER_SLOT = TICKS_PER_ROTATION / 3;

    public static int PRESENCE_THRESHOLD = 150;

    private double currentPower = 0;
    private static double RAMP_RATE = 0.05;

    private final BallColor[] positions = { BallColor.EMPTY, BallColor.EMPTY, BallColor.EMPTY };

    public CarouselSubsystem(HardwareMap hardwareMap) {
        carouselMotor = hardwareMap.dcMotor.get("carousel_motor");
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselMotor.setTargetPosition(0);
        carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftIntake = hardwareMap.dcMotor.get("left_intake");
        rightIntake = hardwareMap.dcMotor.get("right_intake");
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        kickerServo = hardwareMap.get(Servo.class, "flicker_servo");
        kickerServo.setPosition(KICKER_DOWN);

        light1 = hardwareMap.get(Servo.class, "light1");
        light2 = hardwareMap.get(Servo.class, "light2");
        light3 = hardwareMap.get(Servo.class, "light3");
        light1.setPosition(LIGHT_OFF);
        light2.setPosition(LIGHT_OFF);
        light3.setPosition(LIGHT_OFF);

        sensorsA[POS_INTAKE] = hardwareMap.get(RevColorSensorV3.class, "intake_color1");
        sensorsB[POS_INTAKE] = hardwareMap.get(RevColorSensorV3.class, "intake_color2");
        sensorsA[POS_BACK_LEFT] = hardwareMap.get(RevColorSensorV3.class, "BL_color");
        sensorsB[POS_BACK_LEFT] = hardwareMap.get(RevColorSensorV3.class, "BL_upper");
        sensorsA[POS_BACK_RIGHT] = hardwareMap.get(RevColorSensorV3.class, "BR_color");
        sensorsB[POS_BACK_RIGHT] = hardwareMap.get(RevColorSensorV3.class, "BR_upper");
    }

    @Override
    public void periodic() {
        updateMotor();
        updatePositionContents();
        updateLights();
    }

    // Lights

    private void updateLights() {
        light1.setPosition(getLightValue(positions[POS_INTAKE]));
        light2.setPosition(getLightValue(positions[POS_BACK_LEFT]));
        light3.setPosition(getLightValue(positions[POS_BACK_RIGHT]));
    }

    private double getLightValue(BallColor color) {
        switch (color) {
            case GREEN: return LIGHT_GREEN;
            case PURPLE: return LIGHT_PURPLE;
            default: return LIGHT_OFF;
        }
    }

    // Carousel rotation
    // +ticks = forward = INTAKE -> BACK_LEFT -> BACK_RIGHT -> INTAKE

    public void rotateToKicker(BallColor color) {
        int pos = findPositionWithColor(color);
        if (pos == -1) return;
        rotatePositionToKicker(pos);
    }

    public void rotatePositionToKicker(int position) {
        int steps = getStepsToIntake(position);
        int targetTicks = carouselMotor.getCurrentPosition() + (steps * TICKS_PER_SLOT);
        carouselMotor.setTargetPosition(targetTicks);
        currentPower = 0;
    }

    public void rotateEmptyToIntake() {
        int emptyPos = findPositionWithColor(BallColor.EMPTY);
        if (emptyPos == -1) return;
        rotatePositionToKicker(emptyPos);
    }

    public void rotateOneStepForward() {
        int targetTicks = carouselMotor.getCurrentPosition() + TICKS_PER_SLOT;
        carouselMotor.setTargetPosition(targetTicks);
        currentPower = 0;
    }

    public void rotateOneStepBackward() {
        int targetTicks = carouselMotor.getCurrentPosition() - TICKS_PER_SLOT;
        carouselMotor.setTargetPosition(targetTicks);
        currentPower = 0;
    }

    private int getStepsToIntake(int position) {
        switch (position) {
            case POS_INTAKE: return 0;
            case POS_BACK_LEFT: return -1;
            case POS_BACK_RIGHT: return 1;
            default: return 0;
        }
    }

    public boolean isSettled() {
        return !carouselMotor.isBusy();
    }

    public void stop() {
        carouselMotor.setTargetPosition(carouselMotor.getCurrentPosition());
        currentPower = CAROUSEL_POWER;
        carouselMotor.setPower(currentPower);
    }

    private void updateMotor() {
        if (isSettled()) {
            currentPower = CAROUSEL_POWER;
        } else {
            int distanceRemaining = Math.abs(carouselMotor.getTargetPosition() - carouselMotor.getCurrentPosition());
            int rampDownThreshold = TICKS_PER_SLOT / 3;

            if (distanceRemaining < rampDownThreshold) {
                double minPower = 0.1;
                double targetPower = minPower + (CAROUSEL_POWER - minPower) * ((double) distanceRemaining / rampDownThreshold);
                currentPower = Math.max(targetPower, minPower);
            } else {
                if (currentPower < CAROUSEL_POWER) {
                    currentPower = Math.min(currentPower + RAMP_RATE, CAROUSEL_POWER);
                }
            }
        }
        carouselMotor.setPower(currentPower);
    }

    // Intake

    public void runIntake() {
        leftIntake.setPower(INTAKE_POWER);
        rightIntake.setPower(INTAKE_POWER);
    }

    public void reverseIntake() {
        leftIntake.setPower(-INTAKE_POWER);
        rightIntake.setPower(-INTAKE_POWER);
    }

    public void stopIntake() {
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }

    // Kicker

    public void setKickerUp() {
        kickerServo.setPosition(KICKER_UP);
    }

    public void setKickerDown() {
        kickerServo.setPosition(KICKER_DOWN);
    }

    public boolean isKickerDown() {
        return kickerServo.getPosition() == KICKER_DOWN;
    }

    // Position contents

    public BallColor getPositionContents(int position) {
        if (position < 0 || position >= 3) return BallColor.EMPTY;
        return positions[position];
    }

    public BallColor getIntakeContents() {
        return positions[POS_INTAKE];
    }

    public int findPositionWithColor(BallColor color) {
        for (int i = 0; i < 3; i++) {
            if (positions[i] == color) return i;
        }
        return -1;
    }

    public boolean hasColor(BallColor color) {
        return findPositionWithColor(color) != -1;
    }

    public int getBallCount() {
        int count = 0;
        for (BallColor pos : positions) {
            if (pos == BallColor.GREEN || pos == BallColor.PURPLE) count++;
        }
        return count;
    }

    public boolean isFull() {
        return getBallCount() >= 3;
    }

    public boolean isEmpty() {
        return getBallCount() == 0;
    }

    public boolean isIntakeEmpty() {
        return getIntakeContents() == BallColor.EMPTY;
    }

    // Color detection

    private void updatePositionContents() {
        for (int pos = 0; pos < 3; pos++) {
            BallColor detected = detectBallAtPosition(pos);
            if (detected != BallColor.UNKNOWN) {
                positions[pos] = detected;
            }
        }
    }

    private BallColor detectBallAtPosition(int position) {
        ColorSensor sensorA = sensorsA[position];
        ColorSensor sensorB = sensorsB[position];

        if (sensorA == null && sensorB == null) {
            return BallColor.UNKNOWN;
        }

        BallColor typeA = (sensorA != null) ? classifyBall(sensorA) : BallColor.UNKNOWN;
        BallColor typeB = (sensorB != null) ? classifyBall(sensorB) : BallColor.UNKNOWN;

        if (typeA == typeB) return typeA;
        if (typeA == BallColor.UNKNOWN) return typeB;
        if (typeB == BallColor.UNKNOWN) return typeA;
        if (typeA == BallColor.EMPTY) return typeB;
        if (typeB == BallColor.EMPTY) return typeA;

        return typeA;
    }

    private BallColor classifyBall(ColorSensor sensor) {
        int alpha = sensor.alpha();
        if (alpha < PRESENCE_THRESHOLD) {
            return BallColor.EMPTY;
        }

        int blue = sensor.blue();
        int green = sensor.green();

        if (green == 0) {
            return (blue > 0) ? BallColor.PURPLE : BallColor.UNKNOWN;
        }

        double ratio = (double) blue / green;

        // ratio > 1 = purple, ratio < 1 = green
        if (ratio > 1.0) {
            return BallColor.PURPLE;
        } else {
            return BallColor.GREEN;
        }
    }

    // Telemetry getters

    public int getCurrentTicks() {
        return carouselMotor.getCurrentPosition();
    }

    public int getTargetTicks() {
        return carouselMotor.getTargetPosition();
    }

    public void setCarouselPower(double power) {
        CAROUSEL_POWER = Math.abs(power);
    }

    public double getCarouselPower() {
        return CAROUSEL_POWER;
    }

    public void setRampRate(double rate) {
        RAMP_RATE = Math.abs(rate);
    }

    public double getRampRate() {
        return RAMP_RATE;
    }

    public BallColor[] getAllPositions() {
        return positions.clone();
    }
}