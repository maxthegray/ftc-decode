package org.firstinspires.ftc.teamcode.threaded;

import android.util.Size;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class CameraThread extends Thread {

    private final BotState state;
    private final int basketTagId;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    // Tag 24 field position (Red Basket)
    private static final double TAG_24_X = -58.35 - 72;
    private static final double TAG_24_Y = 55.63 - 72;
    private static final double TAG_24_HEADING = -54.0;

    // Tag 20 field position (Blue Basket)
    private static final double TAG_20_X = 58.35 + 72;
    private static final double TAG_20_Y = 55.63 - 72;
    private static final double TAG_20_HEADING = -126.0;

    // Shoot order tags
    private static final int TAG_GPP = 21;  // Green, Purple, Purple
    private static final int TAG_PGP = 22;  // Purple, Green, Purple
    private static final int TAG_PPG = 23;  // Purple, Purple, Green

    // Alliance basket tags
    public static final int TAG_BLUE_BASKET = 20;
    public static final int TAG_RED_BASKET = 24;

    public CameraThread(BotState state, HardwareMap hardwareMap, int basketTagId) {
        this.state = state;
        this.basketTagId = basketTagId;

        // Initialize AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .build();

        // Initialize vision portal
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
        // Wait for camera to be streaming
        while (!state.shouldKillThreads()) {
            VisionPortal.CameraState cameraState = visionPortal.getCameraState();
            state.setCameraState(cameraState.toString());

            if (cameraState == VisionPortal.CameraState.STREAMING) {
                break;
            }

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                return;
            }
        }

        // Main detection loop
        while (!state.shouldKillThreads()) {

            // Get detections
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

            // Update detection count for debugging
            state.setDetectionCount(detections.size());

            // Find relevant tags
            AprilTagDetection basketTag = null;
            AprilTagDetection shootOrderTag = null;

            for (AprilTagDetection detection : detections) {
                if (detection.id == basketTagId && detection.ftcPose != null) {
                    basketTag = detection;
                } else if (detection.id == TAG_GPP || detection.id == TAG_PGP || detection.id == TAG_PPG) {
                    shootOrderTag = detection;
                }
            }

            // Update basket tag state (Tag 24)
            if (basketTag != null) {
                state.setTagData(
                        basketTag.id,
                        basketTag.ftcPose.bearing,
                        basketTag.ftcPose.range
                );

                // Calculate robot pose from tag
                Pose robotPose = calculateRobotPose(basketTag);
                if (robotPose != null) {
                    state.setTagCalculatedPose(robotPose);
                }
            } else {
                state.clearTagData();
                state.setTagCalculatedPose(null);
            }

            // Update shoot order from tags 21, 22, 23
            if (shootOrderTag != null) {
                BotState.BallColor[] order = getShootOrderFromTag(shootOrderTag.id);
                state.setDetectedShootOrder(order, shootOrderTag.id);
            }

            try {
                Thread.sleep(BotState.CAMERA_UPDATE_MS);
            } catch (InterruptedException e) {
                break;
            }
        }

        // Cleanup
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    private Pose calculateRobotPose(AprilTagDetection tag) {
        if (tag == null || tag.ftcPose == null) return null;

        // Get tag position based on which basket we're tracking
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

        double cameraHeadingRad = Math.toRadians(tagHeading - yaw);

        // Normalize heading
        while (cameraHeadingRad > Math.PI) cameraHeadingRad -= 2 * Math.PI;
        while (cameraHeadingRad < -Math.PI) cameraHeadingRad += 2 * Math.PI;

        return new Pose(cameraFieldX, cameraFieldY, cameraHeadingRad);
    }

    private BotState.BallColor[] getShootOrderFromTag(int tagId) {
        switch (tagId) {
            case TAG_GPP:  // 21
                return new BotState.BallColor[] {
                        BotState.BallColor.GREEN,
                        BotState.BallColor.PURPLE,
                        BotState.BallColor.PURPLE
                };
            case TAG_PGP:  // 22
                return new BotState.BallColor[] {
                        BotState.BallColor.PURPLE,
                        BotState.BallColor.GREEN,
                        BotState.BallColor.PURPLE
                };
            case TAG_PPG:  // 23
                return new BotState.BallColor[] {
                        BotState.BallColor.PURPLE,
                        BotState.BallColor.PURPLE,
                        BotState.BallColor.GREEN
                };
            default:
                return null;
        }
    }
}