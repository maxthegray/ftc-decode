package org.firstinspires.ftc.teamcode.commandbased.subsystems;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagSubsystem extends SubsystemBase {

    private final AprilTagProcessor camera;
    private final VisionPortal visionPortal;
    private final Telemetry telemetry;

    // Configurable update rate
    public static long UPDATE_INTERVAL_MS = 100;
    private final ElapsedTime updateTimer = new ElapsedTime();

    // Cached data
    private AprilTagDetection cachedBasketTag = null;
    private double cachedRange = 0;
    private double cachedBearing = 0;
    private Pose cachedRobotPose = null;

    // Tag positions
    private static final double TAG_24_X = -58.35 - 72;
    private static final double TAG_24_Y = 55.63 - 72;
    private static final double TAG_24_HEADING = -54.0;

    public AprilTagSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        camera = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "hsc"))
                .setCameraResolution(new Size(1280, 800))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(false)
                .addProcessor(camera)
                .build();
    }

    @Override
    public void periodic() {
        if (updateTimer.milliseconds() < UPDATE_INTERVAL_MS) {
            return;
        }
        updateTimer.reset();

        // Only process if camera is streaming
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return;
        }

        updateCachedData();
    }

    private void updateCachedData() {
        List<AprilTagDetection> detections = camera.getDetections();
        cachedBasketTag = null;

        for (AprilTagDetection detection : detections) {
            if (detection.id == 24 && detection.ftcPose != null) {
                cachedBasketTag = detection;
                cachedRange = detection.ftcPose.range;
                cachedBearing = detection.ftcPose.bearing;
                cachedRobotPose = calculateRobotPose(detection);
                return;
            }
        }

        cachedRange = 0;
        cachedBearing = 0;
        cachedRobotPose = null;
    }

    private Pose calculateRobotPose(AprilTagDetection tag) {
        double range = tag.ftcPose.range;
        double bearing = tag.ftcPose.bearing;
        double yaw = tag.ftcPose.yaw;

        double bearingRad = Math.toRadians(bearing);
        double tagHeadingRad = Math.toRadians(TAG_24_HEADING);

        double cameraToTagX = range * Math.cos(bearingRad);
        double cameraToTagY = range * Math.sin(bearingRad);

        double cameraFieldX = TAG_24_X - (cameraToTagX * Math.cos(tagHeadingRad) - cameraToTagY * Math.sin(tagHeadingRad));
        double cameraFieldY = TAG_24_Y - (cameraToTagX * Math.sin(tagHeadingRad) + cameraToTagY * Math.cos(tagHeadingRad));

        double cameraHeadingRad = Math.toRadians(TAG_24_HEADING - yaw);

        while (cameraHeadingRad > Math.PI) cameraHeadingRad -= 2 * Math.PI;
        while (cameraHeadingRad < -Math.PI) cameraHeadingRad += 2 * Math.PI;

        return new Pose(cameraFieldX, cameraFieldY, cameraHeadingRad);
    }

    public boolean hasBasketTag() {
        return cachedBasketTag != null;
    }

    public double getTagRange() {
        return cachedRange;
    }

    public double getTagBearing() {
        return cachedBearing;
    }

    public Pose getRobotPoseFromAprilTag() {
        return cachedRobotPose;
    }

    public static void setUpdateInterval(long intervalMs) {
        UPDATE_INTERVAL_MS = intervalMs;
    }
}