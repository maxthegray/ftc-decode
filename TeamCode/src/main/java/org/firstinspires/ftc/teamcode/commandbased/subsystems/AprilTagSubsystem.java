package org.firstinspires.ftc.teamcode.commandbased.subsystems;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    private int orderID;
    private int exposureMS = 10;

    // === OPTIMIZATION: Cache detection result to avoid redundant list iterations ===
    private AprilTagDetection cachedBasketDetection = null;

    private static final double TAG_24_X = -58.35 - 72;  // = -130.35" from center
    private static final double TAG_24_Y = 55.63 - 72;   // = -16.37" from center
    private static final double TAG_24_HEADING = -54.0; // degrees

    // Camera offset from robot center (adjust these for your robot)
    private static final double CAMERA_OFFSET_X = 0.0;
    private static final double CAMERA_OFFSET_Y = 0.0;
    private static final double CAMERA_HEADING_OFFSET = 0.0;

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

        // Configure exposure
        visionPortal.resumeStreaming();
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
    }

    @Override
    public void periodic() {
        // === OPTIMIZATION: Update cached detection once per loop ===
        // This prevents multiple methods from iterating through detections list
        updateCachedDetection();
    }

    private void updateCachedDetection() {
        cachedBasketDetection = null;
        List<AprilTagDetection> currentDetections = camera.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            // Only use Tag 24 (Red Goal) for alignment and localization
            if (detection.id == 24) {
                cachedBasketDetection = detection;
            }
            // Still track order ID from specimen tags
            if (detection.id == 21 || detection.id == 22 || detection.id == 23) {
                orderID = detection.id;
            }
        }
    }

    /**
     * Returns the cached basket detection from the current loop.
     * Call this instead of iterating through detections multiple times.
     */
    public AprilTagDetection getBasketDetection() {
        return cachedBasketDetection;
    }

    public double getDistanceToTag() {
        if (cachedBasketDetection != null) {
            return cachedBasketDetection.ftcPose.y;
        }
        return 0;
    }

    public boolean isAlignedForDistance(double toleranceDegrees) {
        if (cachedBasketDetection == null) {
            return false;
        }
        return Math.abs(cachedBasketDetection.ftcPose.bearing) <= toleranceDegrees;
    }

    public boolean isAlignedForDistance() {
        return isAlignedForDistance(1.0);
    }

    public Pose getRobotPoseFromAprilTag() {
        if (cachedBasketDetection == null || cachedBasketDetection.id != 24) {
            return null;
        }

        AprilTagDetection tag = cachedBasketDetection;

        // Get Tag 24's known field position (field center origin)
        double tagFieldX = TAG_24_X;
        double tagFieldY = TAG_24_Y;
        double tagFieldHeading = TAG_24_HEADING;

        // Get camera's measurements to the tag
        double range = tag.ftcPose.range;
        double bearing = tag.ftcPose.bearing;
        double yaw = tag.ftcPose.yaw;

        // Convert bearing to radians
        double bearingRad = Math.toRadians(bearing);

        // Camera's position relative to the tag (in tag's reference frame)
        double cameraToTagX = range * Math.cos(bearingRad);
        double cameraToTagY = range * Math.sin(bearingRad);

        // Tag's heading in field coordinates (radians)
        double tagHeadingRad = Math.toRadians(tagFieldHeading);

        // Rotate camera-to-tag vector into field coordinates
        double cameraFieldX = tagFieldX - (cameraToTagX * Math.cos(tagHeadingRad) - cameraToTagY * Math.sin(tagHeadingRad));
        double cameraFieldY = tagFieldY - (cameraToTagX * Math.sin(tagHeadingRad) + cameraToTagY * Math.cos(tagHeadingRad));

        // Calculate camera's heading on field
        double cameraHeadingDeg = tagFieldHeading - yaw - CAMERA_HEADING_OFFSET;
        double cameraHeadingRad = Math.toRadians(cameraHeadingDeg);

        // Convert camera position to robot center position
        double robotFieldX = cameraFieldX - (CAMERA_OFFSET_X * Math.cos(cameraHeadingRad) - CAMERA_OFFSET_Y * Math.sin(cameraHeadingRad));
        double robotFieldY = cameraFieldY - (CAMERA_OFFSET_X * Math.sin(cameraHeadingRad) + CAMERA_OFFSET_Y * Math.cos(cameraHeadingRad));

        // Normalize heading to -PI to PI
        while (cameraHeadingRad > Math.PI) cameraHeadingRad -= 2 * Math.PI;
        while (cameraHeadingRad < -Math.PI) cameraHeadingRad += 2 * Math.PI;

        return new Pose(robotFieldX, robotFieldY, cameraHeadingRad);
    }

    public boolean hasBasketTag() {
        return cachedBasketDetection != null;
    }

    public double getTagBearing() {
        return (cachedBasketDetection != null) ? cachedBasketDetection.ftcPose.bearing : 0;
    }

    public double getTagElevation() {
        return (cachedBasketDetection != null) ? cachedBasketDetection.ftcPose.elevation : 0;
    }

    public double getTagRange() {
        return (cachedBasketDetection != null) ? cachedBasketDetection.ftcPose.range : 0;
    }

    public int getOrderID() {
        return orderID;
    }

    public boolean canSeeRedTag() {
        return (cachedBasketDetection != null && cachedBasketDetection.id == 24);
    }

    public void addTelemetry() {
        telemetry.addData("Visible Tags", camera.getDetections().size());
        if (cachedBasketDetection != null) {
            telemetry.addData("Tag ID", cachedBasketDetection.id);
            telemetry.addData("Range", "%.2f inches", cachedBasketDetection.ftcPose.range);
            telemetry.addData("Bearing", "%.2f degrees", cachedBasketDetection.ftcPose.bearing);
            telemetry.addData("Elevation", "%.2f degrees", cachedBasketDetection.ftcPose.elevation);
            telemetry.addData("Distance (Y)", "%.2f inches", getDistanceToTag());
            telemetry.addData("Aligned?", isAlignedForDistance() ? "YES" : "NO");
        } else {
            telemetry.addData("Goal Tag", "Not Detected");
        }
    }
}