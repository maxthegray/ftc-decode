package org.firstinspires.ftc.teamcode.commandbased.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CarouselSubsystem extends SubsystemBase {

    // Hardware
    private final DcMotor carouselMotor;
    private final Servo flickerServo;
    private final DigitalChannel leftFinLimit;
    private final DigitalChannel rightFinLimit;

    // Carousel constants
    private static final int TICKS_PER_REV = 6700 / 4;
    public static final int POSITION_1 = 0;
    public static final int POSITION_2 = TICKS_PER_REV / 3;
    public static final int POSITION_3 = (TICKS_PER_REV / 3) * 2;
    private static final double MOTOR_POWER = 0.6;
    private static final int POSITION_TOLERANCE = 5;

    // Flicker constants
    public static final double FLICKER_DOWN = 0.0;
    public static final double FLICKER_UP = 0.4;

    // State
    private int targetPosition = 0;
    private double flickerPosition = FLICKER_DOWN;

    public CarouselSubsystem(HardwareMap hardwareMap) {
        // initialize carousel motor
        carouselMotor = hardwareMap.dcMotor.get("carouselMotor");
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // initialize flicker servo
        flickerServo = hardwareMap.get(Servo.class, "flickServo");

        // initialize limit switches
        leftFinLimit = hardwareMap.get(DigitalChannel.class, "leftFin");
        leftFinLimit.setMode(DigitalChannel.Mode.INPUT);

        rightFinLimit = hardwareMap.get(DigitalChannel.class, "rightFin");
        rightFinLimit.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void periodic() {
        // update carousel motor
        carouselMotor.setTargetPosition(targetPosition);
        carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carouselMotor.setPower(MOTOR_POWER);

        // update flicker servo
        flickerServo.setPosition(flickerPosition);
    }

// Carousel Methods
    public void setTargetPosition(int position) {
        targetPosition = position;
    }

    public void goToPosition1() {
        targetPosition = POSITION_1;
    }

    public void goToPosition2() {
        targetPosition = POSITION_2;
    }

    public void goToPosition3() {
        targetPosition = POSITION_3;
    }

    public boolean isSettled() {
        int error = Math.abs(carouselMotor.getCurrentPosition() - targetPosition);
        return error <= POSITION_TOLERANCE;
    }

    public int getCurrentPosition() {
        return carouselMotor.getCurrentPosition();
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public double getMotorPower() {
        return carouselMotor.getPower();
    }

    //Flicker Methods

    public boolean flickUp() {
        if (isSettled()) {
            flickerPosition = FLICKER_UP;
            return true;
        }
        return false;
    }

    public boolean flickDown() {
        if (isSettled()) {
            flickerPosition = FLICKER_DOWN;
            return true;
        }
        return false;
    }


    public void setFlickerPosition(double position) {
        flickerPosition = position;
    }

    public double getFlickerPosition() {
        return flickerPosition;
    }

    public boolean isFlickerUp() {
        return flickerPosition == FLICKER_UP;
    }

    public boolean isFlickerDown() {
        return flickerPosition == FLICKER_DOWN;
    }

    // Limit Switch

    public boolean isLeftFinPressed() {
        return !leftFinLimit.getState();
    }

    public boolean isRightFinPressed() {
        return !rightFinLimit.getState();
    }

    public String getLeftFinStatus() {
        return leftFinLimit.getState() ? "P" : "NP";
    }

    public String getRightFinStatus() {
        return rightFinLimit.getState() ? "P" : "NP";
    }
}