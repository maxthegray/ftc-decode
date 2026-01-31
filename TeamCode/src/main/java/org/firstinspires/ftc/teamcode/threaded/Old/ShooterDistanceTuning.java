package org.firstinspires.ftc.teamcode.threaded.Old;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Shooter Distance Tuning", group = "Tuning")
public class ShooterDistanceTuning extends LinearOpMode {

    private DcMotorEx launcherMotor;
    private Servo kickerServo;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private Follower follower;

    // Tuned PID values
    private static final double SHOOTER_P = 200.0;
    private static final double SHOOTER_I = 0.9;
    private static final double SHOOTER_D = 0.1;
    private static final double SHOOTER_F = 0.0;

    // Kicker constants
    private static final double KICKER_DOWN = 0.0;
    private static final double KICKER_UP = 0.4;
    private static final double KICK_DURATION_MS = 250;

    // Target velocity (adjustable)
    private double targetVelocity = 210.0;  // degrees per second
    private boolean motorRunning = false;

    // Adjustment increments
    private static final double COARSE_INCREMENT = 10.0;
    private static final double FINE_INCREMENT = 1.0;

    // AprilTag data
    private double tagRange = 0;
    private double tagBearing = 0;
    private boolean tagVisible = false;

    // Kick state
    private boolean kicking = false;
    private ElapsedTime kickTimer = new ElapsedTime();

    // Last recorded kick data
    private String lastKickData = "No kicks yet";

    // Button edge detection
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevY = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;

    @Override
    public void runOpMode() {
        // Initialize drive
        follower = Constants.createFollower(hardwareMap);
        follower.startTeleOpDrive();

        // Initialize shooter motor
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher_motor");
        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Apply tuned PID coefficients
        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F));

        // Initialize kicker
        kickerServo = hardwareMap.get(Servo.class, "flicker_servo");
        kickerServo.setPosition(KICKER_DOWN);

        // Initialize AprilTag
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "hsc"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new Size(1280, 800))
                .addProcessor(aprilTagProcessor)
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("Left Stick = Drive");
        telemetry.addLine("Right Stick = Rotate");
        telemetry.addLine("A = Start shooter");
        telemetry.addLine("B = Stop shooter");
        telemetry.addLine("Y = KICK (records data)");
        telemetry.addLine("D-pad Up/Down = ±10 deg/s");
        telemetry.addLine("D-pad Left/Right = ±1 deg/s");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            updateAprilTag();
            handleDrive();
            handleControls();
            handleKick();
            updateMotor();
            updateTelemetry();
        }

        // Cleanup
        launcherMotor.setVelocity(0);
        visionPortal.close();
    }

    private void updateAprilTag() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        tagVisible = false;
        for (AprilTagDetection detection : detections) {
            if (detection.id == 24 && detection.ftcPose != null) {
                tagRange = detection.ftcPose.range;
                tagBearing = detection.ftcPose.bearing;
                tagVisible = true;
                break;
            }
        }
    }

    private void handleDrive() {
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x / 2.0;

        follower.setTeleOpDrive(forward, strafe, rotate, true);
        follower.update();
    }

    private void handleControls() {
        // A = Start motor
        if (gamepad1.a && !prevA) {
            motorRunning = true;
        }
        prevA = gamepad1.a;

        // B = Stop motor
        if (gamepad1.b && !prevB) {
            motorRunning = false;
        }
        prevB = gamepad1.b;

        // Y = Kick and record
        if (gamepad1.y && !prevY && !kicking) {
            // Record data at moment of kick
            lastKickData = String.format("%.1f in @ %.0f deg/s", tagRange, targetVelocity);

            // Start kick
            kickerServo.setPosition(KICKER_UP);
            kicking = true;
            kickTimer.reset();
        }
        prevY = gamepad1.y;

        // D-pad Up = Coarse increase
        if (gamepad1.dpad_up && !prevDpadUp) {
            targetVelocity += COARSE_INCREMENT;
        }
        prevDpadUp = gamepad1.dpad_up;

        // D-pad Down = Coarse decrease
        if (gamepad1.dpad_down && !prevDpadDown) {
            targetVelocity = Math.max(0, targetVelocity - COARSE_INCREMENT);
        }
        prevDpadDown = gamepad1.dpad_down;

        // D-pad Right = Fine increase
        if (gamepad1.dpad_right && !prevDpadRight) {
            targetVelocity += FINE_INCREMENT;
        }
        prevDpadRight = gamepad1.dpad_right;

        // D-pad Left = Fine decrease
        if (gamepad1.dpad_left && !prevDpadLeft) {
            targetVelocity = Math.max(0, targetVelocity - FINE_INCREMENT);
        }
        prevDpadLeft = gamepad1.dpad_left;
    }

    private void handleKick() {
        if (kicking && kickTimer.milliseconds() >= KICK_DURATION_MS) {
            kickerServo.setPosition(KICKER_DOWN);
            kicking = false;
        }
    }

    private void updateMotor() {
        if (motorRunning) {
            launcherMotor.setVelocity(targetVelocity, AngleUnit.DEGREES);
        } else {
            launcherMotor.setVelocity(0);
        }
    }

    private void updateTelemetry() {
        telemetry.addLine("=== LAST KICK ===");
        telemetry.addData(">>>", lastKickData);
        telemetry.addLine();

        telemetry.addLine("=== APRILTAG ===");
        if (tagVisible) {
            telemetry.addData("Distance", "%.1f in", tagRange);
            telemetry.addData("Bearing", "%.1f°", tagBearing);
        } else {
            telemetry.addData("Tag 24", "NOT VISIBLE");
        }
        telemetry.addLine();

        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Motor", motorRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Target", "%.1f deg/s", targetVelocity);
        telemetry.addData("Current", "%.1f deg/s", launcherMotor.getVelocity(AngleUnit.DEGREES));
        telemetry.addLine();

        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("Sticks=Drive  A=Start  B=Stop");
        telemetry.addLine("Y=KICK  D-pad=Adjust");

        telemetry.update();
    }
}