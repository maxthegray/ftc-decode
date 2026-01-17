package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Carousel PID Tuning", group = "Tuning")
public class CarouselPIDTuning extends LinearOpMode {

    private DcMotorEx carouselMotor;

    // Calibrated carousel values
    private static final int TICKS_PER_ROTATION = 2332;  // 2332
    private static final int TICKS_PER_SLOT = 780;  // 777

    // Motor power
    private double motorPower = 0.9;

    // PIDF coefficients for RUN_TO_POSITION
    private double kP = 10.0;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kF = 0.0;

    // Adjustment increments
    private static final double COARSE_INCREMENT = 1.0;
    private static final double FINE_INCREMENT = 0.1;
    private static final double POWER_INCREMENT = 0.05;

    // Which coefficient is selected (0=P, 1=I, 2=D, 3=F, 4=Power)
    private int selectedCoeff = 0;
    private final String[] COEFF_NAMES = {"P", "I", "D", "F", "Power"};

    // Button edge detection
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    private boolean prevLBumper = false;
    private boolean prevRBumper = false;
    private boolean prevBack = false;

    @Override
    public void runOpMode() {
        // Initialize motor
        carouselMotor = hardwareMap.get(DcMotorEx.class, "carousel_motor");
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselMotor.setTargetPosition(0);
        carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselMotor.setPower(motorPower);

        // Get current PIDF coefficients for RUN_TO_POSITION
        PIDFCoefficients currentPIDF = carouselMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        kP = currentPIDF.p;
        kI = currentPIDF.i;
        kD = currentPIDF.d;
        kF = currentPIDF.f;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Current PIDF", "P=%.3f I=%.3f D=%.3f F=%.3f", kP, kI, kD, kF);
        telemetry.addData("Ticks/Rotation", TICKS_PER_ROTATION);
        telemetry.addData("Ticks/Slot", TICKS_PER_SLOT);
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("A = Move +1 slot (forward)");
        telemetry.addLine("B = Move -1 slot (backward)");
        telemetry.addLine("X/Y = Select coefficient");
        telemetry.addLine("D-pad Up/Down = Coarse adjust (±1.0)");
        telemetry.addLine("D-pad Left/Right = Fine adjust (±0.1)");
        telemetry.addLine("Bumpers = Apply PIDF to motor");
        telemetry.addLine("Back = Reset encoder to 0");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleControls();
            updateMotor();
            updateTelemetry();
        }

        // Cleanup
        carouselMotor.setPower(0);
    }

    private void handleControls() {
        // A = Move forward one slot
        if (gamepad1.a && !prevA) {
            int newTarget = carouselMotor.getTargetPosition() + TICKS_PER_SLOT;
            carouselMotor.setTargetPosition(newTarget);
        }
        prevA = gamepad1.a;

        // B = Move backward one slot
        if (gamepad1.b && !prevB) {
            int newTarget = carouselMotor.getTargetPosition() - TICKS_PER_SLOT;
            carouselMotor.setTargetPosition(newTarget);
        }
        prevB = gamepad1.b;

        // X = Previous coefficient
        if (gamepad1.x && !prevX) {
            selectedCoeff = (selectedCoeff - 1 + 5) % 5;
        }
        prevX = gamepad1.x;

        // Y = Next coefficient
        if (gamepad1.y && !prevY) {
            selectedCoeff = (selectedCoeff + 1) % 5;
        }
        prevY = gamepad1.y;

        // D-pad Up = Coarse increase
        if (gamepad1.dpad_up && !prevDpadUp) {
            adjustCoefficient(COARSE_INCREMENT);
        }
        prevDpadUp = gamepad1.dpad_up;

        // D-pad Down = Coarse decrease
        if (gamepad1.dpad_down && !prevDpadDown) {
            adjustCoefficient(-COARSE_INCREMENT);
        }
        prevDpadDown = gamepad1.dpad_down;

        // D-pad Right = Fine increase
        if (gamepad1.dpad_right && !prevDpadRight) {
            adjustCoefficient(FINE_INCREMENT);
        }
        prevDpadRight = gamepad1.dpad_right;

        // D-pad Left = Fine decrease
        if (gamepad1.dpad_left && !prevDpadLeft) {
            adjustCoefficient(-FINE_INCREMENT);
        }
        prevDpadLeft = gamepad1.dpad_left;

        // Left Bumper or Right Bumper = Apply PIDF
        if ((gamepad1.left_bumper && !prevLBumper) || (gamepad1.right_bumper && !prevRBumper)) {
            applyPIDF();
        }
        prevLBumper = gamepad1.left_bumper;
        prevRBumper = gamepad1.right_bumper;

        // Back = Reset encoder
        if (gamepad1.back && !prevBack) {
            carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            carouselMotor.setTargetPosition(0);
            carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            carouselMotor.setPower(motorPower);
        }
        prevBack = gamepad1.back;
    }

    private void adjustCoefficient(double delta) {
        switch (selectedCoeff) {
            case 0: kP = Math.max(0, kP + delta); break;
            case 1: kI = Math.max(0, kI + delta); break;
            case 2: kD = Math.max(0, kD + delta); break;
            case 3: kF = Math.max(0, kF + delta); break;
            case 4: motorPower = Math.max(0, Math.min(1.0, motorPower + delta * POWER_INCREMENT)); break;
        }
    }

    private void applyPIDF() {
        PIDFCoefficients newPIDF = new PIDFCoefficients(10, 0.05, 0, 0);
        carouselMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, newPIDF);
        carouselMotor.setPower(motorPower);
    }

    private void updateMotor() {
        carouselMotor.setPower(motorPower);
    }

    private void updateTelemetry() {
        telemetry.addLine("=== CAROUSEL PID TUNING ===");
        telemetry.addLine();

        int currentPos = carouselMotor.getCurrentPosition();
        int targetPos = carouselMotor.getTargetPosition();
        int error = targetPos - currentPos;
        boolean busy = carouselMotor.isBusy();

        telemetry.addData("Status", busy ? "MOVING" : "SETTLED");
        telemetry.addData("Current Pos", "%d ticks", currentPos);
        telemetry.addData("Target Pos", "%d ticks", targetPos);
        telemetry.addData("Error", "%d ticks", error);
        telemetry.addData("Slot Position", "%.2f", (double) currentPos / TICKS_PER_SLOT);
        telemetry.addData("Tolerance", carouselMotor.getTargetPositionTolerance());
        telemetry.addLine();

        telemetry.addLine("=== PIDF COEFFICIENTS ===");
        for (int i = 0; i < 5; i++) {
            String prefix = (i == selectedCoeff) ? ">> " : "   ";
            double value = 0;
            switch (i) {
                case 0: value = kP; break;
                case 1: value = kI; break;
                case 2: value = kD; break;
                case 3: value = kF; break;
                case 4: value = motorPower; break;
            }
            telemetry.addData(prefix + COEFF_NAMES[i], "%.3f", value);
        }
        telemetry.addLine();

        telemetry.addLine("=== CONSTANTS ===");
        telemetry.addData("Ticks/Rotation", TICKS_PER_ROTATION);
        telemetry.addData("Ticks/Slot", TICKS_PER_SLOT);
        telemetry.addLine();

        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("A=+1 slot  B=-1 slot");
        telemetry.addLine("X/Y=Select coeff");
        telemetry.addLine("D-pad U/D=±1.0  L/R=±0.1");
        telemetry.addLine("Bumper=Apply  Back=Reset");

        telemetry.update();
    }
}