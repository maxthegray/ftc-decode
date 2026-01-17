package org.firstinspires.ftc.teamcode.threaded;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class CameraThread extends Thread {

    private final BotState state;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    // Basket tag IDs (into the net tags)
    private static final int[] BASKET_TAG_IDS = { 11, 12, 13, 14 };

    private static final long UPDATE_INTERVAL_MS = 50;

    public CameraThread(BotState state, HardwareMap hardwareMap) {
        this.state = state;

        // Initialize AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
//                .setDrawAxes(true)
//                .setDrawCubeProjection(true)
//                .setDrawTagOutline(true)
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
        visionPortal.resumeStreaming();
    }

    @Override
    public void run() {
        while (!state.shouldKillThreads()) {

            // Get detections
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

            // Find basket tag
            AprilTagDetection basketTag = null;
            for (AprilTagDetection detection : detections) {
                if (isBasketTag(detection.id)) {
                    basketTag = detection;
                    break;
                }
            }

            // Update state
            if (basketTag != null && basketTag.ftcPose != null) {
                state.setTagData(
                        basketTag.id,
                        basketTag.ftcPose.bearing,
                        basketTag.ftcPose.range
                );
            } else {
                state.clearTagData();
            }

            try {
                Thread.sleep(UPDATE_INTERVAL_MS);
            } catch (InterruptedException e) {
                break;
            }
        }

        // Cleanup
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    private boolean isBasketTag(int id) {
        for (int basketId : BASKET_TAG_IDS) {
            if (id == basketId) return true;
        }
        return false;
    }
}