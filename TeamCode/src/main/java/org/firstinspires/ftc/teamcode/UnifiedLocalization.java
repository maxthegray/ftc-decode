
package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.List;

public class UnifiedLocalization {

    private SparkFunOTOS myOtos;
    private double startx;
    private double starty;
    private double startheading;
    private Telemetry telemetry;
// Odometry stuff -------------------
    public UnifiedLocalization(Telemetry telemetryy, HardwareMap hardwareMap, double sx, double sy, double sh) {
        myOtos = hardwareMap.get(SparkFunOTOS.class, "odometrySensor");
        startx = sx;
        starty = sy;
        startheading = sh;
        telemetry = telemetryy;
        configureOtos();

        camera = new AprilTagProcessor.Builder().build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "hsc"));
        builder.setCameraResolution(new Size(1280, 800));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.enableLiveView(true);
        builder.addProcessor(camera);

        visionPortal = builder.build();
    }
    public AprilTagDetection getDetectionById(int targetId) {
        List<AprilTagDetection> currentDetections = camera.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == targetId) {
                return detection;
            }
        }
        return null; // Return null if the tag is not found
    }


    public void configureOtos() {
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);
        myOtos.calibrateImu();
        myOtos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(startx, starty, startheading);
        myOtos.setPosition(currentPosition);

        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);
    }

    public double getOdoX() { return myOtos.getPosition().x; }
    public double getOdoY() { return myOtos.getPosition().y; }
    public double getOdoHeading() { return myOtos.getPosition().h; }

    // AprilTag stuff --------------------------------------

    public int colorID;
    private final AprilTagProcessor camera;
    private final VisionPortal visionPortal;

    private double tagX;
    private double tagY;
    private double tagPitch;
    private double tagHeading;

    public void updateAprilTag() {
        List<AprilTagDetection> detections = camera.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == 23 || detection.id == 22 || detection.id == 21) {
                colorID = detection.id;
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
        tagX = tagFieldX - relX;
        tagY = tagFieldY - relY; // should be  + rotated

        // absolute heading
        tagHeading = tagFieldHeading + detection.ftcPose.yaw;
    }
    public int getID() {
        return camera.getDetections().get(0).id;
    }

    public double getTagX() { return tagX; }
    public double getTagY() { return tagY; }
    public double getTagPitch() { return tagPitch; }
    public double getTagHeading() { return tagHeading; }

    //both stuff -----



    public void step() {
        getDetectionById(24);
//        updateAprilTag();
        sync();

    }





    public void addTelemetry() {
        telemetry.addData("Apr X (in)", tagX);
        telemetry.addData("Apr Y (in)", tagY);
        telemetry.addData("Visible Tags", camera.getDetections().size());

        telemetry.addData("Odo X (in)", getOdoX());
        telemetry.addData("Odo Y (in)", getOdoY());
        telemetry.addData("Odo Heading (deg)", getOdoHeading());

    }

    double absX, absY, absH;

    public void sync() {
        if (canSeeTag()) {
            absX = getTagX();
            absY = getTagY();
            absH = getTagHeading();
        } else {
            absX = getOdoX();
            absY = getOdoY();
            absH = getOdoHeading();
        }
    }


    public boolean canSeeTag() {
        return !camera.getDetections().isEmpty();
    }

    public void stop() {
        visionPortal.close();
    }



}
