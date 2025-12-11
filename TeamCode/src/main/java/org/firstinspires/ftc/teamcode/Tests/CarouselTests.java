package org.firstinspires.ftc.teamcode.Tests;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "CarouselTest", group = "Testing")
public class CarouselTests extends OpMode {

    // Carousel constants and variables
    private DcMotor carouselMotor;
    private int targetPosition = 0;
    private static final int Ticks_per_rev = 6700 / 4;
    private static final int POSITION_1 = 0;
    private static final int POSITION_2 = Ticks_per_rev / 3;
    private static final int POSITION_3 = (Ticks_per_rev / 3) * 2;
    private static final double power = 0.6;
    private static final int POSITION_TOLERANCE = 5;

    // flicker constants and variables
    private Servo flickerServo;
    private double flickerPosition = 0;
    private static final double down = 0;
    private static final double up = 0.4;
    private static final long delay = 50; //ms

    // limit switches
    private DigitalChannel leftFinLimit;
    private DigitalChannel rightFinLimit;

    @Override
    public void init() {
        initCarousel();
        initFlicker();
        initLimitSwitches();
    }

    @Override
    public void loop() {
        handleCarouselInput();
        updateCarousel();

        handleFlickerInput();
        updateFlicker();

        updateTelemetry();
    }

    // init methods

    private void initCarousel() {
        carouselMotor = hardwareMap.dcMotor.get("carouselMotor");
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initFlicker() {
        flickerServo = hardwareMap.get(Servo.class, "flickServo");
        flickerPosition = down;
    }

    private void initLimitSwitches() {
        leftFinLimit = hardwareMap.get(DigitalChannel.class, "leftFin");
        leftFinLimit.setMode(DigitalChannel.Mode.INPUT);

        rightFinLimit = hardwareMap.get(DigitalChannel.class, "rightFin");
        rightFinLimit.setMode(DigitalChannel.Mode.INPUT);
    }

    // carousel methods

    private void handleCarouselInput() {
        if (gamepad1.square) {
            targetPosition = POSITION_1;
        } else if (gamepad1.triangle) {
            targetPosition = POSITION_2;
        } else if (gamepad1.circle) {
            targetPosition = POSITION_3;
        }
    }

    private void updateCarousel() {
        carouselMotor.setTargetPosition(targetPosition);
        carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carouselMotor.setPower(power);
    }

    private boolean isCarouselSettled() {
        int error = Math.abs(carouselMotor.getCurrentPosition() - targetPosition);
        return error <= POSITION_TOLERANCE;
    }

    // flicker methods

    private void handleFlickerInput() {
        if (!isCarouselSettled()) {
            return;
        }

        if (gamepad1.dpad_up) {
            setFlickerPosition(up);
        } else if (gamepad1.dpad_down) {
            setFlickerPosition(down);
        }
    }

    private void setFlickerPosition(double position) {
        flickerPosition = position;
        sleep(delay);
    }

    private void updateFlicker() {
        flickerServo.setPosition(flickerPosition);
    }

    // telemetry

    private void updateTelemetry() {
        telemetry.addData("Servo Position", flickerPosition);
        telemetry.addData("Carousel Motor Power", carouselMotor.getPower());
        telemetry.addData("Carousel Motor Position", carouselMotor.getCurrentPosition());
        telemetry.addData("Carousel Target", targetPosition);
        telemetry.addData("Carousel Settled", isCarouselSettled() ? "Y" : "N");
        telemetry.addData("Left Fin", getLimitSwitchStatus(leftFinLimit));
        telemetry.addData("Right Fin", getLimitSwitchStatus(rightFinLimit));
        telemetry.update();
    }

    private String getLimitSwitchStatus(DigitalChannel limitSwitch) {
        return limitSwitch.getState() ? "P" : "NP";
    }
}