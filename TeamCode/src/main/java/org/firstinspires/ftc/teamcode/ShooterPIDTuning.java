//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//
//@TeleOp(name = "Shooter PID Tuning", group = "Tuning")
//public class ShooterPIDTuning extends LinearOpMode {
//
//    private DcMotorEx carouselMotor;
//
//    // Target velocity
//    private static final double TARGET_VELOCITY = 170;  // degrees per second
//    private boolean motorRunning = false;
//
//    // PIDF coefficients (start with defaults, adjust as needed)
//    private double kP = 200;
//    private double kI = 0.9;
//    private double kD = 0.1;
//    private double kF = 0.0;
//
//    // Adjustment increments
//    private static final double COARSE_INCREMENT = 1.0;
//    private static final double FINE_INCREMENT = 0.1;
//
//    // Which coefficient is selected (0=P, 1=I, 2=D, 3=F)
//    private int selectedCoeff = 0;
//    private final String[] COEFF_NAMES = {"P", "I", "D", "F"};
//
//    // Button edge detection
//    private boolean prevA = false;
//    private boolean prevB = false;
//    private boolean prevX = false;
//    private boolean prevY = false;
//    private boolean prevDpadUp = false;
//    private boolean prevDpadDown = false;
//    private boolean prevDpadLeft = false;
//    private boolean prevDpadRight = false;
//    private boolean prevLBumper = false;
//    private boolean prevRBumper = false;
//
//    @Override
//    public void runOpMode() {
//        // Initialize motor
//        carouselMotor = hardwareMap.get(DcMotorEx.class, "carousel_motor");
//        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        carouselMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        // Get current PIDF coefficients
//        PIDFCoefficients currentPIDF = carouselMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//        kP = currentPIDF.p;
//        kI = currentPIDF.i;
//        kD = currentPIDF.d;
//        kF = currentPIDF.f;
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.addData("Current PIDF", "P=%.3f I=%.3f D=%.3f F=%.3f", kP, kI, kD, kF);
//        telemetry.addLine();
//        telemetry.addLine("Controls:");
//        telemetry.addLine("A = Start motor at 210 deg/s");
//        telemetry.addLine("B = Stop motor");
//        telemetry.addLine("X/Y = Select coefficient");
//        telemetry.addLine("D-pad Up/Down = Coarse adjust (±1.0)");
//        telemetry.addLine("D-pad Left/Right = Fine adjust (±0.1)");
//        telemetry.addLine("Bumpers = Apply PIDF to motor");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            handleControls();
//            updateMotor();
//            updateTelemetry();
//        }
//
//        // Cleanup
//        carouselMotor.setVelocity(0);
//    }
//
//    private void handleControls() {
//        // A = Start motor
//        if (gamepad1.a && !prevA) {
//            motorRunning = true;
//        }
//        prevA = gamepad1.a;
//
//        // B = Stop motor
//        if (gamepad1.b && !prevB) {
//            motorRunning = false;
//        }
//        prevB = gamepad1.b;
//
//        // X = Previous coefficient
//        if (gamepad1.x && !prevX) {
//            selectedCoeff = (selectedCoeff - 1 + 4) % 4;
//        }
//        prevX = gamepad1.x;
//
//        // Y = Next coefficient
//        if (gamepad1.y && !prevY) {
//            selectedCoeff = (selectedCoeff + 1) % 4;
//        }
//        prevY = gamepad1.y;
//
//        // D-pad Up = Coarse increase
//        if (gamepad1.dpad_up && !prevDpadUp) {
//            adjustCoefficient(COARSE_INCREMENT);
//        }
//        prevDpadUp = gamepad1.dpad_up;
//
//        // D-pad Down = Coarse decrease
//        if (gamepad1.dpad_down && !prevDpadDown) {
//            adjustCoefficient(-COARSE_INCREMENT);
//        }
//        prevDpadDown = gamepad1.dpad_down;
//
//        // D-pad Right = Fine increase
//        if (gamepad1.dpad_right && !prevDpadRight) {
//            adjustCoefficient(FINE_INCREMENT);
//        }
//        prevDpadRight = gamepad1.dpad_right;
//
//        // D-pad Left = Fine decrease
//        if (gamepad1.dpad_left && !prevDpadLeft) {
//            adjustCoefficient(-FINE_INCREMENT);
//        }
//        prevDpadLeft = gamepad1.dpad_left;
//
//        // Left Bumper or Right Bumper = Apply PIDF
//        if ((gamepad1.left_bumper && !prevLBumper) || (gamepad1.right_bumper && !prevRBumper)) {
//            applyPIDF();
//        }
//        prevLBumper = gamepad1.left_bumper;
//        prevRBumper = gamepad1.right_bumper;
//    }
//
//    private void adjustCoefficient(double delta) {
//        switch (selectedCoeff) {
//            case 0: kP = Math.max(0, kP + delta); break;
//            case 1: kI = Math.max(0, kI + delta); break;
//            case 2: kD = Math.max(0, kD + delta); break;
//            case 3: kF = Math.max(0, kF + delta); break;
//        }
//    }
//
//    private void applyPIDF() {
//        PIDFCoefficients newPIDF = new PIDFCoefficients(kP, kI, kD, kF);
//        carouselMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, newPIDF);
//    }
//
//
//    private void updateTelemetry() {
//        telemetry.addLine("=== SHOOTER PID TUNING ===");
//        telemetry.addLine();
//
//        telemetry.addData("Motor", motorRunning ? "RUNNING" : "STOPPED");
//        telemetry.addData("Target", "%.1f deg/s", TARGET_VELOCITY);
//        telemetry.addData("Current", "%.1f deg/s", carouselMotor.getVelocity(AngleUnit.DEGREES));
//        telemetry.addData("Error", "%.1f deg/s", TARGET_VELOCITY - carouselMotor.getVelocity(AngleUnit.DEGREES));
//        telemetry.addLine();
//
//        telemetry.addLine("=== PIDF COEFFICIENTS ===");
//        for (int i = 0; i < 4; i++) {
//            String prefix = (i == selectedCoeff) ? ">> " : "   ";
//            double value = 0;
//            switch (i) {
//                case 0: value = kP; break;
//                case 1: value = kI; break;
//                case 2: value = kD; break;
//                case 3: value = kF; break;
//            }
//            telemetry.addData(prefix + COEFF_NAMES[i], "%.3f", value);
//        }
//
//        telemetry.update();
//    }
//}