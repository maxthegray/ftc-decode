package org.firstinspires.ftc.teamcode.commandbased.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CarouselSubsystem extends SubsystemBase {

    public enum BallColor { GREEN, PURPLE, EMPTY, UNKNOWN }

    public static final int POS_INTAKE = 0;
    public static final int POS_BACK_LEFT = 1;
    public static final int POS_BACK_RIGHT = 2;

    private final DcMotorEx carouselMotor;
    private final DcMotorEx leftIntake;
    private final DcMotorEx rightIntake;
    private final Servo kickerServo;
    private final CRServo intakeServo;  // New CR servo for intake

    private final Servo light1, light2, light3;

    private final RevColorSensorV3[] sensorsA = new RevColorSensorV3[3];
    private final RevColorSensorV3[] sensorsB = new RevColorSensorV3[3];

    private static final double LIGHT_OFF = 0.0;
    private static final double LIGHT_GREEN = 0.500;
    private static final double LIGHT_PURPLE = 0.722;
    private static final double LIGHTS_DURATION = 3.0; // seconds

    private final ElapsedTime lightsTimer = new ElapsedTime();
    private boolean lightsActive = false;

    private static double CAROUSEL_POWER = 0.6;
    private static final double INTAKE_POWER = 0.75;
    private static final double INTAKE_SERVO_POWER = 1.0;  // CR servo power
    private static final double KICKER_DOWN = 0.0;
    private static final double KICKER_UP = 0.4;

    public static int TICKS_PER_ROTATION = 2230;
    public static int TICKS_PER_SLOT = TICKS_PER_ROTATION / 3;

    // Per-position alpha thresholds for ball detection
    public static int THRESHOLD_INTAKE = 200;
    public static int THRESHOLD_BACK_LEFT = 200;
    public static int THRESHOLD_BACK_RIGHT = 200;

    private final BallColor[] positions = { BallColor.EMPTY, BallColor.EMPTY, BallColor.EMPTY };

    // Configurable update rates (milliseconds between updates)
    public static long SENSOR_UPDATE_INTERVAL_MS = 100;   // Color sensors (I2C is slow)
    public static long MOTOR_UPDATE_INTERVAL_MS = 100;    // Motor/servo updates

    private final ElapsedTime sensorUpdateTimer = new ElapsedTime();
    private final ElapsedTime motorUpdateTimer = new ElapsedTime();

    public CarouselSubsystem(HardwareMap hardwareMap) {
        carouselMotor = hardwareMap.get(DcMotorEx.class, "carousel_motor");
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselMotor.setTargetPosition(0);
        carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carouselMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(10, 0, 0, 0));
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftIntake = hardwareMap.get(DcMotorEx.class, "left_intake");
        rightIntake = hardwareMap.get(DcMotorEx.class, "right_intake");
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        kickerServo = hardwareMap.get(Servo.class, "flicker_servo");
        kickerServo.setPosition(KICKER_DOWN);

        intakeServo = hardwareMap.get(CRServo.class, "intake_servo");

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
        // Motor updates (faster)
        if (motorUpdateTimer.milliseconds() >= MOTOR_UPDATE_INTERVAL_MS) {
            motorUpdateTimer.reset();
            updateMotor();
            updateLights();
        }

        // Sensor updates (slower - I2C is the bottleneck)
        if (sensorUpdateTimer.milliseconds() >= SENSOR_UPDATE_INTERVAL_MS) {
            sensorUpdateTimer.reset();
            updatePositionContents();
        }
    }

    // Lights

    private void updateLights() {
        // Turn off lights if timer expired
        if (lightsActive && lightsTimer.seconds() >= LIGHTS_DURATION) {
            lightsActive = false;
        }

        if (lightsActive) {
            light1.setPosition(getLightValue(positions[POS_INTAKE]));
            light2.setPosition(getLightValue(positions[POS_BACK_LEFT]));
            light3.setPosition(getLightValue(positions[POS_BACK_RIGHT]));
        } else {
            light1.setPosition(LIGHT_OFF);
            light2.setPosition(LIGHT_OFF);
            light3.setPosition(LIGHT_OFF);
        }
    }

    public void showLights() {
        lightsTimer.reset();
        lightsActive = true;
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
    }

    public void rotateEmptyToIntake() {
        int emptyPos = findPositionWithColor(BallColor.EMPTY);
        if (emptyPos == -1) return;
        rotatePositionToKicker(emptyPos);
    }

    public void rotateOneStepForward() {
        int targetTicks = carouselMotor.getCurrentPosition() + TICKS_PER_SLOT;
        carouselMotor.setTargetPosition(targetTicks);
    }

    public void rotateOneStepBackward() {
        int targetTicks = carouselMotor.getCurrentPosition() - TICKS_PER_SLOT;
        carouselMotor.setTargetPosition(targetTicks);
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
    }

    private void updateMotor() {
        carouselMotor.setPower(CAROUSEL_POWER);
    }

    // Intake

    public void runIntake() {
        leftIntake.setPower(INTAKE_POWER);
        rightIntake.setPower(INTAKE_POWER);
        intakeServo.setPower(INTAKE_SERVO_POWER);
    }

    public void reverseIntake() {
        leftIntake.setPower(-INTAKE_POWER);
        rightIntake.setPower(-INTAKE_POWER);
        intakeServo.setPower(-INTAKE_SERVO_POWER);
    }

    public void stopIntake() {
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        intakeServo.setPower(0);
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

        int threshold = getThresholdForPosition(position);

        BallColor typeA = (sensorA != null) ? classifyBall(sensorA, threshold) : BallColor.UNKNOWN;
        BallColor typeB = (sensorB != null) ? classifyBall(sensorB, threshold) : BallColor.UNKNOWN;

        if (typeA == typeB) return typeA;
        if (typeA == BallColor.UNKNOWN) return typeB;
        if (typeB == BallColor.UNKNOWN) return typeA;
        if (typeA == BallColor.EMPTY) return typeB;
        if (typeB == BallColor.EMPTY) return typeA;

        return typeA;
    }

    private int getThresholdForPosition(int position) {
        switch (position) {
            case POS_INTAKE: return THRESHOLD_INTAKE;
            case POS_BACK_LEFT: return THRESHOLD_BACK_LEFT;
            case POS_BACK_RIGHT: return THRESHOLD_BACK_RIGHT;
            default: return 200;
        }
    }

    private BallColor classifyBall(ColorSensor sensor, int threshold) {
        int alpha = sensor.alpha();
        if (alpha < threshold) {
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

    public void setThreshold(int position, int threshold) {
        switch (position) {
            case POS_INTAKE: THRESHOLD_INTAKE = threshold; break;
            case POS_BACK_LEFT: THRESHOLD_BACK_LEFT = threshold; break;
            case POS_BACK_RIGHT: THRESHOLD_BACK_RIGHT = threshold; break;
        }
    }

    public int getThreshold(int position) {
        return getThresholdForPosition(position);
    }

    // Raw sensor data for calibration
    public int getSensorAlpha(int position, boolean useSensorA) {
        ColorSensor sensor = useSensorA ? sensorsA[position] : sensorsB[position];
        return (sensor != null) ? sensor.alpha() : 0;
    }

    public int getSensorBlue(int position, boolean useSensorA) {
        ColorSensor sensor = useSensorA ? sensorsA[position] : sensorsB[position];
        return (sensor != null) ? sensor.blue() : 0;
    }

    public int getSensorGreen(int position, boolean useSensorA) {
        ColorSensor sensor = useSensorA ? sensorsA[position] : sensorsB[position];
        return (sensor != null) ? sensor.green() : 0;
    }

    public BallColor[] getAllPositions() {
        return positions.clone();
    }

    // Configurable update rates
    public static void setSensorUpdateInterval(long intervalMs) {
        SENSOR_UPDATE_INTERVAL_MS = intervalMs;
    }

    public static long getSensorUpdateInterval() {
        return SENSOR_UPDATE_INTERVAL_MS;
    }

    public static void setMotorUpdateInterval(long intervalMs) {
        MOTOR_UPDATE_INTERVAL_MS = intervalMs;
    }

    public static long getMotorUpdateInterval() {
        return MOTOR_UPDATE_INTERVAL_MS;
    }
}