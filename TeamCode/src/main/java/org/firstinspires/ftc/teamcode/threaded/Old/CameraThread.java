package org.firstinspires.ftc.teamcode.threaded.Old;

import android.util.Size;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Handles AprilTag detection.
 */
public class CameraThread extends Thread {

    private final       SensorState state;
    private final int basketTagId;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    // Camera mounting offset (radians) — negate if heading goes the wrong way
    private static final double CAMERA_HEADING_OFFSET = Math.PI / 2;  // 90° correction

    // Tag positions
    private static final double TAG_24_X = -58.35 - 72;
    private static final double TAG_24_Y = 55.63 - 72;
    private static final double TAG_24_HEADING = -54.0;

    private static final double TAG_20_X = 58.35 + 72;
    private static final double TAG_20_Y = 55.63 - 72;
    private static final double TAG_20_HEADING = -126.0;

    // Shoot order tags
    private static final int TAG_GPP = 21;
    private static final int TAG_PGP = 22;
    private static final int TAG_PPG = 23;

    // Alliance tags
    public static final int TAG_BLUE_BASKET = 20;
    public static final int TAG_RED_BASKET = 24;

    public CameraThread(      SensorState state, HardwareMap hardwareMap, int basketTagId) {
        this.state = state;
        this.basketTagId = basketTagId;

        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "hsc"))
                .addProcessor(aprilTagProcessor)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new Size(1280, 800))
                .enableLiveView(false)
                .setAutoStartStreamOnBuild(true)
                .build();
    }

    @Override
    public void run() {
        // Wait for camera
        while (!state.shouldKillThreads()) {
            if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                break;
            }
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                return;
            }
        }

        // Main loop
        while (!state.shouldKillThreads()) {
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

            AprilTagDetection basketTag = null;
            AprilTagDetection shootOrderTag = null;

            for (AprilTagDetection det : detections) {
                if (det.id == basketTagId && det.ftcPose != null) {
                    basketTag = det;
                } else if (det.id == TAG_GPP || det.id == TAG_PGP || det.id == TAG_PPG) {
                    shootOrderTag = det;
                }
            }

            // Update basket tag
            if (basketTag != null) {
                state.setTagData(
                        basketTag.id,
                        basketTag.ftcPose.bearing,
                        basketTag.ftcPose.range,
                        basketTag.ftcPose.yaw
                );

                Pose robotPose = calculateRobotPose(basketTag);
                if (robotPose != null) {
                    state.setTagCalculatedPose(robotPose);
                }
            } else {
                state.clearTagData();
                state.setTagCalculatedPose(null);
            }

            // Update shoot order
            if (shootOrderTag != null) {
                state.setDetectedShootOrder(getShootOrderFromTag(shootOrderTag.id));
            }

            try {
                Thread.sleep(      SensorState.CAMERA_UPDATE_MS);
            } catch (InterruptedException e) {
                break;
            }
        }

        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    private Pose calculateRobotPose(AprilTagDetection tag) {
        if (tag == null || tag.ftcPose == null) return null;

        double tagX, tagY, tagHeading;
        if (basketTagId == TAG_BLUE_BASKET) {
            tagX = TAG_20_X;
            tagY = TAG_20_Y;
            tagHeading = TAG_20_HEADING;
        } else {
            tagX = TAG_24_X;
            tagY = TAG_24_Y;
            tagHeading = TAG_24_HEADING;
        }

        double range = tag.ftcPose.range;
        double bearing = tag.ftcPose.bearing;
        double yaw = tag.ftcPose.yaw;

        double bearingRad = Math.toRadians(bearing);
        double tagHeadingRad = Math.toRadians(tagHeading);

        double cameraToTagX = range * Math.cos(bearingRad);
        double cameraToTagY = range * Math.sin(bearingRad);

        double cameraFieldX = tagX - (cameraToTagX * Math.cos(tagHeadingRad) - cameraToTagY * Math.sin(tagHeadingRad));
        double cameraFieldY = tagY - (cameraToTagX * Math.sin(tagHeadingRad) + cameraToTagY * Math.cos(tagHeadingRad));

        double cameraHeadingRad = Math.toRadians(tagHeading - yaw) + CAMERA_HEADING_OFFSET;

        while (cameraHeadingRad > Math.PI) cameraHeadingRad -= 2 * Math.PI;
        while (cameraHeadingRad < -Math.PI) cameraHeadingRad += 2 * Math.PI;

        return new Pose(cameraFieldX, cameraFieldY, cameraHeadingRad);
    }

    private       ShootSequence.BallColor[] getShootOrderFromTag(int tagId) {
        switch (tagId) {
            case TAG_GPP:
                return new       ShootSequence.BallColor[] {
                        ShootSequence.BallColor.GREEN,
                        ShootSequence.BallColor.PURPLE,
                        ShootSequence.BallColor.PURPLE
                };
            case TAG_PGP:
                return new       ShootSequence.BallColor[] {
                        ShootSequence.BallColor.PURPLE,
                        ShootSequence.BallColor.GREEN,
                        ShootSequence.BallColor.PURPLE
                };
            case TAG_PPG:
                return new       ShootSequence.BallColor[] {
                        ShootSequence.BallColor.PURPLE,
                        ShootSequence.BallColor.PURPLE,
                        ShootSequence.BallColor.GREEN
                };
            default:
                return null;
        }
    }
}