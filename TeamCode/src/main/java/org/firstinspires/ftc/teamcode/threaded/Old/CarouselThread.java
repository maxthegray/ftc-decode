package org.firstinspires.ftc.teamcode.threaded.Old;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.threaded.Old.BotState.BallColor;
import org.firstinspires.ftc.teamcode.threaded.Old.BotState.CarouselCommand;

public class CarouselThread extends Thread {

    private final BotState state;

    // Hardware
    private final DcMotorEx carouselMotor;
    private final DcMotorEx leftIntake;
    private final DcMotorEx rightIntake;
    private final Servo kickerServo;
    private final CRServo intakeServo;
    private final Servo light1, light2, light3;

    // Constants
    private static final double CAROUSEL_POWER = 0.9;
    private static final double INTAKE_POWER = 0.75;
    private static final double INTAKE_SERVO_POWER = 1.0;
    private static final double KICKER_DOWN = 0.0;
    private static final double KICKER_UP = 0.4;

    private static final double LIGHT_OFF = 0.0;
    private static final double LIGHT_GREEN = 0.500;
    private static final double LIGHT_PURPLE = 0.722;
    private static final double LIGHTS_DURATION = 3.0;

    private static final double KICK_DURATION_MS = BotState.SEQ_KICK_MS;

    private static final double POST_KICK_DURATION_MS = BotState.SEQ_POST_KICK_MS;


    // State
    private final ElapsedTime kickTimer = new ElapsedTime();
    private final ElapsedTime lightsTimer = new ElapsedTime();
    private boolean kicking = false;
    private boolean lightsActive = false;

    // Auto-index state
    private boolean ballWasInIntake = false;

    public CarouselThread(BotState state, HardwareMap hardwareMap) {
        this.state = state;
        this.setPriority(Thread.NORM_PRIORITY);

        // Carousel motor
        carouselMotor = hardwareMap.get(DcMotorEx.class, "carousel_motor");
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselMotor.setTargetPosition(0);
        carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselMotor.setPower(CAROUSEL_POWER);

        // Intake motors
        leftIntake = hardwareMap.get(DcMotorEx.class, "left_intake");
        rightIntake = hardwareMap.get(DcMotorEx.class, "right_intake");
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        // Kicker
        kickerServo = hardwareMap.get(Servo.class, "flicker_servo");
        kickerServo.setPosition(KICKER_DOWN);

        // Intake CR servo
        intakeServo = hardwareMap.get(CRServo.class, "intake_servo");

        // Lights
        light1 = hardwareMap.get(Servo.class, "light1");
        light2 = hardwareMap.get(Servo.class, "light2");
        light3 = hardwareMap.get(Servo.class, "light3");
        light1.setPosition(LIGHT_OFF);
        light2.setPosition(LIGHT_OFF);
        light3.setPosition(LIGHT_OFF);
    }

    @Override
    public void run() {
        while (!state.shouldKillThreads()) {

            // Handle carousel commands
            handleCarouselCommand();

            // Handle kick request
            handleKick();

            // Handle intake
            handleIntake();

            // Handle lights
            handleLights();

            // Auto-index logic
            handleAutoIndex();

            // Update state
            state.setCarouselCurrentTicks(carouselMotor.getCurrentPosition());
            state.setCarouselSettled(!carouselMotor.isBusy());

            try {
                Thread.sleep(BotState.CAROUSEL_UPDATE_MS);
            } catch (InterruptedException e) {
                break;
            }
        }

        // Cleanup
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        intakeServo.setPower(0);
    }

    private void handleCarouselCommand() {
        CarouselCommand cmd = state.getCarouselCommand();
        if (cmd == CarouselCommand.NONE) return;

        int currentTarget = carouselMotor.getTargetPosition();
        int targetTicks = currentTarget;

        switch (cmd) {
            case ROTATE_EMPTY_TO_INTAKE:
                int emptyPos = state.findPositionWithColor(BallColor.EMPTY);
                if (emptyPos != -1) {
                    targetTicks = currentTarget + (-getStepsToIntake(emptyPos) * BotState.TICKS_PER_SLOT);
                }
                break;

            case ROTATE_LEFT:
                targetTicks = currentTarget - BotState.TICKS_PER_SLOT;
                break;

            case ROTATE_RIGHT:
                targetTicks = currentTarget + BotState.TICKS_PER_SLOT;
                break;
        }

            carouselMotor.setTargetPosition(targetTicks);
            state.setCarouselTargetTicks(targetTicks);
            state.clearCarouselCommand();


    }

    private int getStepsToIntake(int position) {
        switch (position) {
            case BotState.POS_INTAKE: return 0;
            case BotState.POS_BACK_LEFT: return -1;
            case BotState.POS_BACK_RIGHT: return 1;
            default: return 0;
        }
    }

    private void handleKick() {
        // Only kick if shooter is ready
        if (state.isKickRequested() && !kicking && state.isCarouselSettled() && state.isShooterReady()) {
            // Start kick
            kickerServo.setPosition(KICKER_UP);
            state.setKickerUp(true);
            kicking = true;
            kickTimer.reset();
            state.clearKickRequest();
        }

        if (kicking && kickTimer.milliseconds() >= KICK_DURATION_MS) {
            // End kick
            kickerServo.setPosition(KICKER_DOWN);
            state.setKickerUp(false);
            kicking = false;
        }

    }

    private void handleIntake() {
        if (state.isIntakeForward()) {
            leftIntake.setPower(INTAKE_POWER);
            rightIntake.setPower(INTAKE_POWER);
            intakeServo.setPower(INTAKE_SERVO_POWER);
        } else if (state.isIntakeReverse()) {
            leftIntake.setPower(-INTAKE_POWER);
            rightIntake.setPower(-INTAKE_POWER);
            intakeServo.setPower(-INTAKE_SERVO_POWER);
        } else {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
            intakeServo.setPower(0);
        }
    }

    private void handleLights() {
        // Check for show lights request
        if (state.isShowLightsRequested()) {
            lightsTimer.reset();
            lightsActive = true;
            state.clearShowLightsRequest();
        }

        // Turn off after duration
        if (lightsActive && lightsTimer.seconds() >= LIGHTS_DURATION) {
            lightsActive = false;
        }

        // Update lights
        if (lightsActive) {
            light1.setPosition(getLightValue(state.getPositionColor(BotState.POS_INTAKE)));
            light2.setPosition(getLightValue(state.getPositionColor(BotState.POS_BACK_LEFT)));
            light3.setPosition(getLightValue(state.getPositionColor(BotState.POS_BACK_RIGHT)));
        } else {
            light1.setPosition(LIGHT_OFF);
            light2.setPosition(LIGHT_OFF);
            light3.setPosition(LIGHT_OFF);
        }
    }

    private double getLightValue(BallColor color) {
        switch (color) {
            case GREEN: return LIGHT_GREEN;
            case PURPLE: return LIGHT_PURPLE;
            default: return LIGHT_OFF;
        }
    }

    private void handleAutoIndex() {
        // Always track ball presence at intake, regardless of other conditions
        BallColor intakeColor = state.getPositionColor(BotState.POS_INTAKE);
        boolean hasBall = (intakeColor == BallColor.GREEN || intakeColor == BallColor.PURPLE);

        // Detect rising edge: ball just arrived at intake
        boolean ballJustArrived = hasBall && !ballWasInIntake;

        // Update tracking state BEFORE any early returns
        ballWasInIntake = hasBall;

        // Now check if we should actually trigger the index
        if (!state.isAutoIndexEnabled()) return;
        if (!state.isCarouselSettled()) return;
        if (state.isFull()) return;

        if (ballJustArrived) {
            // Ball just arrived, rotate empty to intake
            state.setCarouselCommand(CarouselCommand.ROTATE_EMPTY_TO_INTAKE);
        }
    }
}