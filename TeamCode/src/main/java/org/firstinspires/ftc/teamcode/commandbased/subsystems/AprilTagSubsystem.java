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

    // AprilTag field positions (from FTC game manual - DECODE 2025-2026)
    // Origin at FIELD CENTER (0, 0)
    // All positions are in inches, angles in degrees
    // Field coordinate system: X = left/right, Y = forward/back, Heading = rotation

    // We only use Tag 24 (Red Goal) for alignment and localization
    // Tag 20 (Blue Goal) is ignored

    // Red Alliance Goal Tag (ONLY ONE WE USE)
    // From game manual: Field position X=-58.35", Y=55.63" (if corner is origin)
    // Converting to center origin: Need to subtract field center offset
    // FTC field is 144" x 144" (12ft x 12ft)
    // Center offset = 144 / 2 = 72"
    private static final double TAG_24_X = -58.35 - 72;  // = -128.97" from center
    private static final double TAG_24_Y = 55.63 - 72;   // = -14.99" from center
    private static final double TAG_24_HEADING = -54.0; // degrees

    // Camera offset from robot center (adjust these for your robot)
    // Positive X = camera is forward of center
    // Positive Y = camera is left of center
    // Positive heading offset = camera pointing left relative to robot front
    private static final double CAMERA_OFFSET_X = 0.0;  // inches - TODO: MEASURE THIS
    private static final double CAMERA_OFFSET_Y = 0.0;  // inches - TODO: MEASURE THIS
    private static final double CAMERA_HEADING_OFFSET = 0.0; // degrees - TODO: MEASURE THIS

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

    public AprilTagDetection getBasketDetection() {
        List<AprilTagDetection> currentDetections = camera.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Only use Tag 24 (Red Goal) for alignment and localization
            if (detection.id == 24) {
                return detection;
            }
            // Still track order ID from specimen tags (not used in DECODE)
            if (detection.id == 21 || detection.id == 22 || detection.id == 23) {
                orderID = detection.id;
            }
        }
        return null;
    }

    /**
     * Get the distance from the camera to the AprilTag.
     * This should be called when the robot is aligned (bearing near 0) for accurate readings.
     *
     * @return Distance in inches from camera to tag center, or 0 if no tag detected
     */
    public double getDistanceToTag() {
        AprilTagDetection tag = getBasketDetection();
        if (tag != null) {
            // ftcPose.range gives the direct 3D distance from camera to tag center
            return tag.ftcPose.range;
        }
        return 0;
    }

    /**
     * Check if robot is aligned well enough to get accurate distance measurement.
     *
     * @param toleranceDegrees Maximum bearing deviation allowed
     * @return true if aligned within tolerance
     */
    public boolean isAlignedForDistance(double toleranceDegrees) {
        AprilTagDetection tag = getBasketDetection();
        if (tag == null) {
            return false;
        }
        return Math.abs(tag.ftcPose.bearing) <= toleranceDegrees;
    }

    /**
     * Check if robot is aligned well enough to get accurate distance measurement.
     * Uses default tolerance of 1.0 degrees.
     *
     * @return true if aligned within default tolerance
     */
    public boolean isAlignedForDistance() {
        return isAlignedForDistance(1.0);
    }

    /**
     * Calculate the robot's pose on the field using AprilTag detection.
     * This uses Tag 24's known field position and the camera's detection to
     * calculate where the robot is on the field.
     * Origin is at FIELD CENTER (0, 0).
     *
     * @return Pose object with robot's estimated field position (X, Y, Heading in radians),
     *         or null if Tag 24 not detected
     */
    public Pose getRobotPoseFromAprilTag() {
        AprilTagDetection tag = getBasketDetection();
        if (tag == null || tag.id != 24) {
            return null;
        }

        // Get Tag 24's known field position (field center origin)
        double tagFieldX = TAG_24_X;
        double tagFieldY = TAG_24_Y;
        double tagFieldHeading = TAG_24_HEADING;

        // Get camera's measurements to the tag
        double range = tag.ftcPose.range;           // Distance from camera to tag
        double bearing = tag.ftcPose.bearing;       // Horizontal angle from camera axis
        double yaw = tag.ftcPose.yaw;              // Tag's rotation relative to camera

        // Calculate camera position relative to tag
        // We need to work backwards: we know where the tag is, and where the camera
        // sees the tag, so we can figure out where the camera (and robot) is

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
        // The camera's heading is the tag's heading minus the yaw angle
        double cameraHeadingDeg = tagFieldHeading - yaw - CAMERA_HEADING_OFFSET;
        double cameraHeadingRad = Math.toRadians(cameraHeadingDeg);

        // Now convert camera position to robot center position
        // Robot center is offset from camera by CAMERA_OFFSET_X and CAMERA_OFFSET_Y
        double robotFieldX = cameraFieldX - (CAMERA_OFFSET_X * Math.cos(cameraHeadingRad) - CAMERA_OFFSET_Y * Math.sin(cameraHeadingRad));
        double robotFieldY = cameraFieldY - (CAMERA_OFFSET_X * Math.sin(cameraHeadingRad) + CAMERA_OFFSET_Y * Math.cos(cameraHeadingRad));

        // Normalize heading to -PI to PI
        while (cameraHeadingRad > Math.PI) cameraHeadingRad -= 2 * Math.PI;
        while (cameraHeadingRad < -Math.PI) cameraHeadingRad += 2 * Math.PI;

        return new Pose(robotFieldX, robotFieldY, cameraHeadingRad);
    }

    /**
     * Check if the AprilTag pose estimate is reliable enough to use for localization update.
     *
     * @return true if we should trust this pose estimate
     */

    /**
     * Calculate a confidence score for the AprilTag pose estimate (0-1).
     * Higher values mean more confidence.
     * Used for passive sensor fusion with OTOS.
     *
     * @return Confidence from 0 (no confidence) to 1 (high confidence)
     */
    public double getPoseEstimateConfidence() {
        AprilTagDetection tag = getBasketDetection();
        if (tag == null) {
            return 0.0;
        }

        double range = tag.ftcPose.range;
        double bearing = Math.abs(tag.ftcPose.bearing);

        // Confidence decreases with distance and bearing angle
        // Perfect score at 0 distance and 0 bearing, drops off as they increase
        double rangeConfidence = Math.max(0, 1.0 - (range / 100.0));      // 1.0 at 0", 0.0 at 100"
        double bearingConfidence = Math.max(0, 1.0 - (bearing / 45.0));  // 1.0 at 0°, 0.0 at 45°

        // Combined confidence (average of both factors)
        double confidence = (rangeConfidence + bearingConfidence) / 2.0;

        return Math.max(0, Math.min(1.0, confidence));  // Clamp to 0-1
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

    public void addTelemetry() {
        AprilTagDetection tag = getBasketDetection();
        telemetry.addData("Visible Tags", camera.getDetections().size());
        if (tag != null) {
            telemetry.addData("Tag ID", tag.id);
            telemetry.addData("Range", "%.2f inches", tag.ftcPose.range);
            telemetry.addData("Bearing", "%.2f degrees", tag.ftcPose.bearing);
            telemetry.addData("Elevation", "%.2f degrees", tag.ftcPose.elevation);
            telemetry.addData("Distance (when aligned)", "%.2f inches", getDistanceToTag());
            telemetry.addData("Aligned for distance?", isAlignedForDistance() ? "YES" : "NO");
        } else {
            telemetry.addData("Goal Tag", "Not Detected");
        }
    }
}