package org.firstinspires.ftc.teamcode.commandbased.subsystems;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTagSubsystem extends SubsystemBase {

    private final AprilTagProcessor camera;
    private final VisionPortal visionPortal;
    private final Telemetry telemetry;
    private final Servo cameraMount;

    private int orderID;
    private int exposureMS = 10;

    public AprilTagSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize AprilTag processor
        camera = new AprilTagProcessor.Builder().build();

        // Build vision portal
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "hsc"));
        builder.setCameraResolution(new Size(1280, 800));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.enableLiveView(false);
        builder.setAutoStartStreamOnBuild(true);
        builder.addProcessor(camera);

        visionPortal = builder.build();

        // Initialize camera servo
        cameraMount = hardwareMap.get(Servo.class, "cameraServo");
        cameraMount.setPosition(0.15);

        // Configure exposure
        visionPortal.resumeStreaming();
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
    }

    public AprilTagDetection getBasketDetection() {
        List<AprilTagDetection> currentDetections = camera.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 24 || detection.id == 20) {
                return detection;
            }
            if (detection.id == 21 || detection.id == 22 || detection.id == 23) {
                orderID = detection.id;
                return null;
            }
        }
        return null;
    }

    public boolean hasBasketTag() {
        return getBasketDetection() != null;
    }

    public double getTagBearing() {
        AprilTagDetection tag = getBasketDetection();
        return (tag != null) ? tag.ftcPose.bearing : 0;
    }

    public double getTagElevation() {
        AprilTagDetection tag = getBasketDetection();
        return (tag != null) ? tag.ftcPose.elevation : 0;
    }

    public double getTagRange() {
        AprilTagDetection tag = getBasketDetection();
        return (tag != null) ? tag.ftcPose.range : 0;
    }

    public int getOrderID() {
        return orderID;
    }

    public boolean canSeeRedTag() {
        AprilTagDetection tag = getBasketDetection();
        return (tag != null && tag.id == 24);
    }

    public boolean canSeeBlueTag() {
        AprilTagDetection tag = getBasketDetection();
        return (tag != null && tag.id == 20);
    }

    public Servo getCameraMount() {
        return cameraMount;
    }

    public void addTelemetry() {
        AprilTagDetection tag = getBasketDetection();
        telemetry.addData("Visible Tags", camera.getDetections().size());
        if (tag != null) {
            telemetry.addData("Tag ID", tag.id);
            telemetry.addData("Range", "%.2f inches", tag.ftcPose.range);
            telemetry.addData("Bearing", "%.2f degrees", tag.ftcPose.bearing);
            telemetry.addData("Elevation", "%.2f degrees", tag.ftcPose.elevation);
        } else {
            telemetry.addData("Basket Tag", "Not Detected");
        }
    }
}
