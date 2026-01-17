package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.MalformedJsonException;
import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Align and Shoot Auto", group = "Auto")
public class AutonTest extends LinearOpMode {

    // Drive (for alignment only)
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    // Shooter
    private DcMotorEx launcherMotor;
    private static final double SHOOTER_P = 200.0;
    private static final double SHOOTER_I = 0.9;
    private static final double SHOOTER_D = 0.1;
    private static final double SHOOTER_F = 0.0;
    private static final double VELOCITY_TOLERANCE = 50.0;

    // Carousel
    private DcMotorEx carouselMotor;
    private static final int TICKS_PER_SLOT = 780;
    private static final double CAROUSEL_POWER = 0.9;

    // Kicker
    private Servo kickerServo;
    private static final double KICKER_DOWN = 0.0;
    private static final double KICKER_UP = 0.4;
    private static final double KICK_DURATION_MS = 250;

    // Color sensors
    private ColorSensor intakeColorA, intakeColorB;
    private ColorSensor backLeftColorA, backLeftColorB;
    private ColorSensor backRightColorA, backRightColorB;

    // Color sensor thresholds (calibrated)
    private static final int THRESHOLD_INTAKE_A = 80;
    private static final int THRESHOLD_INTAKE_B = 120;
    private static final int THRESHOLD_BACK_LEFT_A = 300;
    private static final int THRESHOLD_BACK_LEFT_B = 200;
    private static final int THRESHOLD_BACK_RIGHT_A = 300;
    private static final int THRESHOLD_BACK_RIGHT_B = 200;

    // Ball positions
    private static final int POS_INTAKE = 0;
    private static final int POS_BACK_LEFT = 1;
    private static final int POS_BACK_RIGHT = 2;

    // Ball colors
    private enum BallColor { GREEN, PURPLE, EMPTY, UNKNOWN }

    // Camera
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private static final int BASKET_TAG_ID = 20;

    // Shoot order tags
    private static final int TAG_GPP = 21;
    private static final int TAG_PGP = 22;
    private static final int TAG_PPG = 23;

    // Default shoot order
    private static final BallColor[] DEFAULT_SHOOT_ORDER = { BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE };

    // Alignment
    private static final double ALIGN_POWER = 0.3;
    private static final double BEARING_TOLERANCE = 2.0;

    // Timers
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize drive (for alignment rotation)
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize shooter
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher_motor");
        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F));

        // Initialize carousel
        carouselMotor = hardwareMap.get(DcMotorEx.class, "carousel_motor");
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselMotor.setTargetPosition(0);
        carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselMotor.setPower(CAROUSEL_POWER);

        // Initialize kicker
        kickerServo = hardwareMap.get(Servo.class, "flicker_servo");
        kickerServo.setPosition(KICKER_DOWN);

        // Initialize color sensors
        intakeColorA = hardwareMap.get(ColorSensor.class, "intake_color1");
        intakeColorB = hardwareMap.get(ColorSensor.class, "intake_color2");
        backLeftColorA = hardwareMap.get(ColorSensor.class, "BL_color");
        backLeftColorB = hardwareMap.get(ColorSensor.class, "BL_upper");
        backRightColorA = hardwareMap.get(ColorSensor.class, "BR_color");
        backRightColorB = hardwareMap.get(ColorSensor.class, "BR_upper");

        // Initialize camera
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "hsc"))
                .setCameraResolution(new Size(1280, 800))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTagProcessor)
                .build();

        telemetry.addData("Status", "Initialized - Align and Shoot");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // === PHASE 1: Align to tag 20 ===
            telemetry.addData("Phase", "Aligning to tag 20");
            telemetry.update();

            alignToTag();

            // === PHASE 2: Shoot 3 sorted balls ===
            telemetry.addData("Phase", "Shooting");
            telemetry.update();

            shootAllBalls();

            telemetry.addData("Status", "Complete!");
            telemetry.update();

            sleep(2000);
        }

        // Cleanup
        launcherMotor.setVelocity(0);
        visionPortal.close();
    }

    // ========================= ALIGNMENT =========================

    private void alignToTag() {
        timer.reset();
        double maxAlignTime = 5.0;  // seconds

        // Start teleop drive mode for rotation control
        follower.startTeleOpDrive();

        while (opModeIsActive() && timer.seconds() < maxAlignTime) {
            AprilTagDetection tag = getTag(BASKET_TAG_ID);

            if (tag != null) {
                double bearing = tag.ftcPose.bearing;

                telemetry.addData("Tag 20", "VISIBLE");
                telemetry.addData("Bearing", "%.1f°", bearing);
                telemetry.addData("Range", "%.1f in", tag.ftcPose.range);

                if (Math.abs(bearing) < BEARING_TOLERANCE) {
                    // Aligned!
                    telemetry.addData("Status", "ALIGNED!");
                    telemetry.update();
                    stopDrive();
                    sleep(200);
                    break;
                }

                // Proportional rotation to align
                double rotate = bearing * 0.02;  // P-control

                telemetry.addData("Rotate Power", "%.2f", rotate);
                telemetry.update();

                follower.setTeleOpDrive(0, 0, rotate, false);
                follower.update();
            } else {
                telemetry.addData("Tag 20", "NOT VISIBLE");
                telemetry.addData("Status", "Searching...");
                telemetry.update();

                stopDrive();
            }

            sleep(20);
        }

        stopDrive();
        telemetry.addData("Align", "Complete (%.1f sec)", timer.seconds());
        telemetry.update();
    }

    private void stopDrive() {
        follower.setTeleOpDrive(0, 0, 0, false);
        follower.update();
    }

    private AprilTagDetection getTag(int tagId) {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == tagId && detection.ftcPose != null) {
                return detection;
            }
        }
        return null;
    }

    private AprilTagDetection getShootOrderTag() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == TAG_GPP || detection.id == TAG_PGP || detection.id == TAG_PPG) {
                return detection;
            }
        }
        return null;
    }

    // ========================= SHOOTING =========================

    private void shootAllBalls() {
        // Get shoot order from tag (21, 22, or 23) or use default
        BallColor[] shootOrder = DEFAULT_SHOOT_ORDER;
        String shootOrderStr = "Default (GPP)";

        AprilTagDetection orderTag = getShootOrderTag();
        if (orderTag != null) {
            switch (orderTag.id) {
                case TAG_GPP:
                    shootOrder = new BallColor[] { BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE };
                    shootOrderStr = "GPP (Tag 21)";
                    break;
                case TAG_PGP:
                    shootOrder = new BallColor[] { BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE };
                    shootOrderStr = "PGP (Tag 22)";
                    break;
                case TAG_PPG:
                    shootOrder = new BallColor[] { BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN };
                    shootOrderStr = "PPG (Tag 23)";
                    break;
            }
        }

        // Get distance and calculate velocity from tag 20
        double targetVelocity = 210;  // Default
        AprilTagDetection tag = getTag(BASKET_TAG_ID);
        if (tag != null) {
            targetVelocity = calculateVelocity(tag.ftcPose.range);
            telemetry.addData("Distance", "%.1f in", tag.ftcPose.range);
        }

        telemetry.addData("Shoot Order", shootOrderStr);
        telemetry.addData("Target Velocity", "%.0f deg/s", targetVelocity);
        telemetry.update();

        // Spin up shooter
        launcherMotor.setVelocity(targetVelocity, AngleUnit.DEGREES);

        // Wait for shooter to reach speed
        timer.reset();
        while (opModeIsActive() && timer.seconds() < 2.0) {
            double currentVel = launcherMotor.getVelocity(AngleUnit.DEGREES);
            telemetry.addData("Shooter", "%.0f / %.0f deg/s", currentVel, targetVelocity);
            telemetry.update();
            if (Math.abs(currentVel - targetVelocity) < VELOCITY_TOLERANCE) {
                break;
            }
            sleep(20);
        }

        // Shoot balls in order
        for (int i = 0; i < shootOrder.length && opModeIsActive(); i++) {
            BallColor targetColor = shootOrder[i];

            // Read current ball positions
            BallColor[] positions = readAllPositions();

            telemetry.addData("Shooting", "Ball %d/3 - Looking for %s", i + 1, targetColor);
            telemetry.addData("Positions", "Intake:%s BL:%s BR:%s",
                    positions[POS_INTAKE], positions[POS_BACK_LEFT], positions[POS_BACK_RIGHT]);
            telemetry.update();

            // Find where this color is
            int ballPosition = findPositionWithColor(positions, targetColor);

            if (ballPosition == -1) {
                // Don't have this color, skip
                telemetry.addData("Skipping", "No %s ball found", targetColor);
                telemetry.update();
                sleep(200);
                continue;
            }

            // Rotate carousel to bring ball to kick position (intake)
            if (ballPosition != POS_INTAKE) {
                int slots = getStepsToIntake(ballPosition);
                telemetry.addData("Rotating", "%d slots to bring %s to intake", slots, targetColor);
                telemetry.update();

                rotateCarousel(slots);
                waitForCarousel();
                sleep(200);  // Let things settle
            }

            // Kick
            kick();
        }

        // Stop shooter
        launcherMotor.setVelocity(0);
    }

    // ========================= COLOR SENSING =========================

    private BallColor[] readAllPositions() {
        BallColor[] positions = new BallColor[3];
        positions[POS_INTAKE] = classifyPosition(
                intakeColorA.alpha(), intakeColorA.blue(), intakeColorA.green(), THRESHOLD_INTAKE_A,
                intakeColorB.alpha(), intakeColorB.blue(), intakeColorB.green(), THRESHOLD_INTAKE_B);
        positions[POS_BACK_LEFT] = classifyPosition(
                backLeftColorA.alpha(), backLeftColorA.blue(), backLeftColorA.green(), THRESHOLD_BACK_LEFT_A,
                backLeftColorB.alpha(), backLeftColorB.blue(), backLeftColorB.green(), THRESHOLD_BACK_LEFT_B);
        positions[POS_BACK_RIGHT] = classifyPosition(
                backRightColorA.alpha(), backRightColorA.blue(), backRightColorA.green(), THRESHOLD_BACK_RIGHT_A,
                backRightColorB.alpha(), backRightColorB.blue(), backRightColorB.green(), THRESHOLD_BACK_RIGHT_B);
        return positions;
    }

    private BallColor classifyPosition(int alphaA, int blueA, int greenA, int thresholdA,
                                       int alphaB, int blueB, int greenB, int thresholdB) {
        BallColor typeA = classifyBall(alphaA, blueA, greenA, thresholdA);
        BallColor typeB = classifyBall(alphaB, blueB, greenB, thresholdB);

        // Merge readings
        if (typeA == typeB) return typeA;
        if (typeA == BallColor.UNKNOWN) return typeB;
        if (typeB == BallColor.UNKNOWN) return typeA;
        if (typeA == BallColor.EMPTY) return typeB;
        if (typeB == BallColor.EMPTY) return typeA;
        return typeA;
    }

    private BallColor classifyBall(int alpha, int blue, int green, int threshold) {
        if (alpha < threshold) {
            return BallColor.EMPTY;
        }
        if (blue > green) {
            return BallColor.PURPLE;
        } else {
            return BallColor.GREEN;
        }
    }

    private int findPositionWithColor(BallColor[] positions, BallColor color) {
        for (int i = 0; i < 3; i++) {
            if (positions[i] == color) return i;
        }
        return -1;
    }

    private int getStepsToIntake(int position) {
        switch (position) {
            case POS_INTAKE: return 0;
            case POS_BACK_LEFT: return -1;
            case POS_BACK_RIGHT: return 1;
            default: return 0;
        }
    }

    // ========================= MECHANICS =========================

    private double calculateVelocity(double distance) {
        // Quartic formula from BotState - calibrated values
        double velocity = 0.00000043 * Math.pow(distance, 4)
                - 0.0001927 * Math.pow(distance, 3)
                + 0.026899 * Math.pow(distance, 2)
                - 0.402824 * distance
                + 136.48202;
        return Math.max(0, velocity);
    }

    private void kick() {
        kickerServo.setPosition(KICKER_UP);
        sleep((long) KICK_DURATION_MS);
        kickerServo.setPosition(KICKER_DOWN);
        sleep(200);  // Wait for kicker to return
    }

    private void rotateCarousel(int slots) {
        int currentTarget = carouselMotor.getTargetPosition();
        int newTarget = currentTarget + (slots * TICKS_PER_SLOT);
        carouselMotor.setTargetPosition(newTarget);
    }

    private void waitForCarousel() {
        timer.reset();
        while (opModeIsActive() && carouselMotor.isBusy() && timer.seconds() < 2.0) {
            sleep(20);
        }
    }
}