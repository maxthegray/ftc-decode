package org.firstinspires.ftc.teamcode.Localization;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class LocalizationCamera {

    public LocalizationCamera(Telemetry telemetryy, HardwareMap hardwaremap) {

        telemetry = telemetryy;

        camera = new AprilTagProcessor.Builder().build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwaremap.get(WebcamName.class, "hsc"));
        builder.setCameraResolution(new Size(1280, 800));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.enableLiveView(true);
        builder.addProcessor(camera);

        visionPortal = builder.build();

    }

    public int colorID;
    private final AprilTagProcessor camera;
    private final VisionPortal visionPortal;

    private double tagX = 0;
    private double tagY = 0;
    private double tagPitch;
    private double tagHeading = 0;

    Telemetry telemetry;

    public void updateAprilTag() {
        List<AprilTagDetection> detections = camera.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == 23 || detection.id == 22 || detection.id == 21) {
                return;
            } else if (detection.id == 20 || detection.id == 24) {
                tagToCoords(detection);
            }
        }
    }


//20 (Blue)	-1.482m	-1.413m	0.749m	54°
//24 (Red)	-1.482m	1.413m	0.749m	-54°

//20 (Blue) -58.35in    -55.63in    29.49in    54°
//24 (Red)  -58.35in     55.63in    29.49in   -54°

    private static final double[] TAG_20_POSE = {-58.35, -55.63, 54.0};
    private static final double[] TAG_24_POSE = {-58.35, 55.63, -54.0};

    //rotate and whereami
    public void tagToCoords(AprilTagDetection detection) {

        //pos of apriltag
        double[] tagPose = (detection.id == 20) ? TAG_20_POSE : TAG_24_POSE;
        double tagFieldX = tagPose[0];
        double tagFieldY = tagPose[1];
        double tagFieldHeading = tagPose[2]; // in degrees

        // robo pos relative to tag, (in tags FOR)
        // ftcPose.x is strafe (sideways) ftcPose.y is range (forward/backward from tag) i think
        double relX = detection.ftcPose.x;
        double relY = detection.ftcPose.y;


        // convert the tags heading to radians for math purposes
        double tagHeadingRad = Math.toRadians(tagFieldHeading);
        double sinTagHeading = Math.sin(tagHeadingRad);
        double cosTagHeading = Math.cos(tagHeadingRad);

        //rotate x and y from tag perspective to field perspecitve
        double rotatedX = relX * cosTagHeading - relY * sinTagHeading;
        double rotatedY = relX * sinTagHeading + relY * cosTagHeading;

        // rotated robo distances + tag field pos = robo field pos (still centered around cam)
        tagX = rotatedX + tagFieldX;
        tagY = rotatedY + tagFieldY; // should be  + rotated

        // absolute heading
        tagHeading = tagFieldHeading + detection.ftcPose.yaw;
    }

    public boolean tagInView() {
        return !camera.getDetections().isEmpty();
    }

    public double getX() { return tagX; }
    public double getY() { return tagY; }
    public double getHeading() { return tagHeading; }


}
