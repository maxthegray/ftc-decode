
package org.firstinspires.ftc.teamcode.Shooter;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.List;
import java.util.concurrent.TimeUnit;

public class ShooterCamera {

    // Odometry stuff -------------------

    private final AprilTagProcessor camera;
    private final VisionPortal visionPortal;
    private Telemetry telemetry;
    private double distanceToTag;
    private double tagElevation;
    boolean colorsAssigned;
    private int orderID;
    private int exposureMS = 100;

    Servo cameraMount;

    public ShooterCamera(Telemetry telemetryy, HardwareMap hardwaremap) {


        telemetry = telemetryy;
        camera = new AprilTagProcessor.Builder().build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwaremap.get(WebcamName.class, "hsc"));
        builder.setCameraResolution(new Size(1280, 800));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.enableLiveView(false);
        builder.addProcessor(camera);

        colorsAssigned = false;


        visionPortal = builder.build();

        cameraMount = hardwaremap.get(Servo.class, "cameraServo");

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
    }



    public void alignCameraToTag() {

        AprilTagDetection lockedTag = getBasketDetection();
        double initialPos = cameraMount.getPosition();
        double targetPos;
        double FPS;

        if (lockedTag != null) {

            targetPos = Range.clip(initialPos + lockedTag.ftcPose.elevation/6.28318530718, 0, 1);
            FPS = (double) 1000 /System.currentTimeMillis();
            telemetry.addData("Elevation", lockedTag.ftcPose.elevation);
            telemetry.addData("Exposure Time", System.currentTimeMillis());
            telemetry.addData("FPS", FPS);
            telemetry.addData("Target Pos", targetPos);


            cameraMount.setPosition(targetPos);
    }

    }
    public double getServoPos() {
        return cameraMount.getPosition();
    }
    public double alignRobotToTagPower() {
        AprilTagDetection lockedTag = getBasketDetection();
        double tolerance = 20; // degrees
        if (lockedTag != null) {
            double bearingDifference = lockedTag.ftcPose.bearing;
            if (bearingDifference > 0) {
                if (bearingDifference > tolerance) {
                    return .2;
                }

            } else if (bearingDifference < 0) {
                if (bearingDifference < -tolerance) {
                    return -.2;
                }

            }
        }
        return 0;
    }

    public AprilTagDetection getBasketDetection() {
        List<AprilTagDetection> currentDetections = camera.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 24 || detection.id == 20) {
                return detection;
            }
//            if (detection.id == 21 || detection.id == 22 || detection.id == 23) {
//                orderID = detection.id;
//            }
            return null; // Return null if the tag is not found
        }
        return null; // Return null if the tag is not found

    }


    //20 (Blue)	-1.482m	-1.413m	0.749m	54°
    //24 (Red)	-1.482m	1.413m	0.749m	-54°

    //20 (Blue) -58.35in    -55.63in    29.49in    54°
    //24 (Red)  -58.35in     55.63in    29.49in   -54°

    private static final double[] TAG_20_POSE = {-58.35, -55.63, 54.0};
    private static final double[] TAG_24_POSE = {-58.35, 55.63, -54.0};

    //rotate and whereami

    public int getID() {
        return camera.getDetections().get(0).id;
    }

    public int getOrderID() { return orderID; }
    public double getTagElevation() { return tagElevation; }
    public double getFromTagDistance() { return distanceToTag; }

    public void addTelemetry() {
        telemetry.addData("Distance to apriltag", distanceToTag);
        telemetry.addData("Visible Tags", camera.getDetections().size());
    }
}
