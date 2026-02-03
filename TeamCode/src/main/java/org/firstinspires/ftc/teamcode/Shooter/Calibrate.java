package org.firstinspires.ftc.teamcode.Shooter;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.old.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@Disabled
@TeleOp(name = "Shooter Calibration", group = "Calibration")
public class Calibrate extends LinearOpMode {

    private Robot robot;
    private Follower follower;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private double shooterPower = 0.0;
    private boolean autoAlignEnabled = false;
    private int carouselSlot = 0;

    private double flickerPosition;
    private double FLICKER_UP = 0.4;
    private double FLICKER_DOWN = 0;

    private static final int TICKS_PER_SLOT = 2230/3; // Adjust after testing
    private static final double ALIGN_POWER = 0.01;
    private static final double BEARING_TOLERANCE = 1.0;

    private ElapsedTime debounce = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize robot and drive
        robot = new Robot(hardwareMap);
        robot.init();

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleOpDrive();

        // Initialize vision
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "hsc"))
                .setCameraResolution(new Size(1280, 800))
                .enableLiveView(false)
                .addProcessor(aprilTagProcessor)
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        setExposure(10);

        waitForStart();

        boolean isLauncherOff = false;

        while (opModeIsActive()) {
            // Driving
            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double rotate = -gamepad1.right_stick_x;

            // Auto-align override
            if (autoAlignEnabled) {
                AprilTagDetection tag = getTag();
                if (tag != null && Math.abs(tag.ftcPose.bearing) > BEARING_TOLERANCE) {
                    rotate =  ALIGN_POWER * getTagBearing(tag)/10;
                }
            }

            follower.setTeleOpDrive(forward, strafe, rotate);
            follower.update();

            // Shooter power (D-pad)
                if (gamepad2.dpad_up) { shooterPower += 1; sleep(100); isLauncherOff = false;}
                if (gamepad2.dpad_down) { shooterPower -= 1; sleep(100); isLauncherOff = false;}
                if (gamepad2.dpad_right) { shooterPower += 5;sleep(100); isLauncherOff = false;}
                if (gamepad2.dpad_left) { shooterPower -= 5; sleep(100); isLauncherOff = false;}
                if (gamepad1.left_bumper) { autoAlignEnabled = !autoAlignEnabled; sleep(100); }
                if (gamepad2.cross) {robot.setLauncherPiwer(0); isLauncherOff = true;}
                if (gamepad2.square) { kick(); }
                if (!isLauncherOff) {
                    robot.setLauncherMotor(shooterPower);
                }


            if (gamepad2.right_trigger > 0.1) robot.setIntakeMotors(30);
            else if (gamepad2.left_trigger > 0.1) robot.setIntakeMotors(-30);
            else if (gamepad2.right_bumper) robot.setIntakeMotors(0);

            // Telemetry
            AprilTagDetection tag = getTag();
            telemetry.addLine(" SHOOTER CALIBRATION ");
            telemetry.addData("Degrees/Second", "%.2f", shooterPower);
            telemetry.addData("DISTANCE", tag != null ? String.format("%.1f in", tag.ftcPose.y) : "NO TAG");
            telemetry.addData("Auto-align", autoAlignEnabled ? "ON" : "OFF");
            telemetry.addData("Bearing", tag!= null ? getTagBearing(tag) : "no tag");

            telemetry.addLine();
            telemetry.update();
        }

        visionPortal.close();
    }

    private AprilTagDetection getTag() {
        for (AprilTagDetection d : aprilTagProcessor.getDetections()) {
            if (d.id == 20) return d; //blue basket, red basket is 24
        }
        return null;
    }

    private double getTagBearing(AprilTagDetection tag) {
        return tag.ftcPose.bearing;
    }

    private void indexCarousel() {
        carouselSlot = (carouselSlot + 1) % 3;
        robot.carouselMotor.setTargetPosition(carouselSlot * TICKS_PER_SLOT);
        robot.carouselMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION);
        robot.carouselMotor.setPower(0.4);
    }

    private void kick() {
        robot.kicker.setPosition(Robot.FLICKER_UP);
        sleep(300);
        robot.kicker.setPosition(Robot.FLICKER_DOWN);
    }


    private void setExposure(int ms) {
        try {
            ExposureControl ctrl = visionPortal.getCameraControl(ExposureControl.class);
            ctrl.setMode(ExposureControl.Mode.Manual);
            ctrl.setExposure(ms, TimeUnit.MILLISECONDS);
        } catch (Exception e) { }
    }


}