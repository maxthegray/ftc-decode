package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;

@TeleOp
public class SpinTest extends LinearOpMode {

    private DcMotorEx carouselMotor;

    private Gamepad currentGamepad1, previousGamepad1;

    // Gaussian constants (tunable)
    private final double A = 0.9;
    private double B = 380;
    private double C = 200.0;
    private int threshold = 760;

    private double stepB = 10.0;
    private double stepC = 10.0;
    private int stepThreshold = 10;

    private enum Param { B, C, THRESHOLD }
    private Param selected = Param.B;

    @Override
    public void runOpMode() {

        carouselMotor = hardwareMap.get(DcMotorEx.class, "carousel_motor");
        carouselMotor.setDirection(DcMotor.Direction.FORWARD);
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

//        telemetry.addLine("DPAD L/R: Select param");
//        telemetry.addLine("DPAD U/D: Adjust value");
//        telemetry.addLine("Bumpers: L=fine, R=coarse");
//        telemetry.addLine("Y: Run test | A: Reset encoder");
//        telemetry.addLine("Right Stick: Manual control");
//        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            // Select parameter
            if (pressed(currentGamepad1.dpad_left, previousGamepad1.dpad_left)) {
                selected = Param.values()[(selected.ordinal() + 2) % 3];
            }
            if (pressed(currentGamepad1.dpad_right, previousGamepad1.dpad_right)) {
                selected = Param.values()[(selected.ordinal() + 1) % 3];
            }

            // Step size: L bumper = fine, R bumper = coarse, neither = medium
            if (gamepad1.left_bumper) {
                stepB = 1.0; stepC = 1.0; stepThreshold = 1;
            } else if (gamepad1.right_bumper) {
                stepB = 50.0; stepC = 50.0; stepThreshold = 50;
            } else {
                stepB = 10.0; stepC = 10.0; stepThreshold = 10;
            }

            // Adjust value
            if (pressed(currentGamepad1.dpad_up, previousGamepad1.dpad_up)) {
                adjust(true);
            }
            if (pressed(currentGamepad1.dpad_down, previousGamepad1.dpad_down)) {
                adjust(false);
            }

            // Manual control
            float stickY = -gamepad1.right_stick_y;
            if (abs(stickY) > 0.05f) {
                carouselMotor.setPower(Range.clip(stickY, -1.0, 1.0));
            } else {
                carouselMotor.setPower(0.0);
            }

            // Reset encoder
            if (pressed(currentGamepad1.a, previousGamepad1.a)) {
                carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // Run Gaussian test
            if (pressed(currentGamepad1.y, previousGamepad1.y)) {
                runGaussianTest();
            }

            // Telemetry
            String bMark = (selected == Param.B) ? ">> " : "   ";
            String cMark = (selected == Param.C) ? ">> " : "   ";
            String tMark = (selected == Param.THRESHOLD) ? ">> " : "   ";

            telemetry.addLine("=== CAROUSEL TUNING ===");
            telemetry.addData(bMark + "B (center)", "%.1f", B);
            telemetry.addData(cMark + "C (width)", "%.1f", C);
            telemetry.addData(tMark + "Threshold", threshold);
            telemetry.addLine();
            telemetry.addData("Step", "%.0f", stepB);
            telemetry.addData("Encoder", carouselMotor.getCurrentPosition());
            telemetry.addLine();
            telemetry.addLine("Y=Run | A=Reset | Stick=Manual");
            telemetry.update();
        }
    }

    private void runGaussianTest() {
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            int pos = carouselMotor.getCurrentPosition();

            if (pos >= threshold) break;
            if (gamepad1.b) break;  // Abort with B

            double power = gaussian(pos);
            carouselMotor.setPower(power);

            telemetry.addData("Position", "%d / %d", pos, threshold);
            telemetry.addData("Power", "%.3f", power);
            telemetry.addLine(" B to abort");
            telemetry.update();
        }
        carouselMotor.setPower(0.0);
    }

    private double gaussian(double x) {
        return A * Math.exp(-Math.pow(x - B, 2) / (2 * Math.pow(C, 2)));
    }

    private void adjust(boolean increase) {
        switch (selected) {
            case B:
                B += increase ? stepB : -stepB;
                B = Math.max(0, B);
                break;
            case C:
                C += increase ? stepC : -stepC;
                C = Math.max(1, C);
                break;
            case THRESHOLD:
                threshold += increase ? stepThreshold : -stepThreshold;
                threshold = Math.max(1, threshold);
                break;
        }
    }

    private boolean pressed(boolean current, boolean previous) {
        return current && !previous;
    }
}