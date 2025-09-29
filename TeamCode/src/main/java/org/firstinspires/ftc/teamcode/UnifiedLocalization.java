
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
    private org.firstinspires.ftc.robotcore.external.Telemetry telemetry;
// Odometry stuff -------------------
    public UnifiedLocalization(Telemetry telemetryy, HardwareMap hardwareMap, double sx, double sy, double sh) {
        myOtos = hardwareMap.get(SparkFunOTOS.class, "odometrySensor");
        startx = sx;
        starty = sy;
        startheading = sh;
        telemetry = telemetryy;
        configureOtos();

        aprilTag = new AprilTagProcessor.Builder().build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "hsc"));
        builder.setCameraResolution(new Size(1280, 800));
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.enableLiveView(true);
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
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
    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

    private double x = 0;
    private double y = 0;
    private double heading = 0;

    public void updateAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == 23 || detection.id == 22 || detection.id == 21) {
                colorID = detection.id;
            } else {
                x = detection.ftcPose.x;
                y = detection.ftcPose.y;
                heading = detection.ftcPose.yaw;
            }
        }
    }

    public int getID() {
        return aprilTag.getDetections().get(0).id;
    }

    public double getTagX() { return x; }
    public double getTagY() { return y; }
    public double getTagHeading() { return heading; }

    //both stuff -----



    public void step() {
        updateAprilTag();
        sync();
    }



    public void addTelemetry() {
        telemetry.addData("Loc X (in)", x);
        telemetry.addData("Loc Y (in)", y);
        telemetry.addData("Heading (deg)", heading);
        telemetry.addData("Visible Tags", aprilTag.getDetections().size());

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
            myOtos.setPosition(new SparkFunOTOS.Pose2D(absX, absY, absH));
        } else {
            absX = getOdoX();
            absY = getOdoY();
            absH = getOdoHeading();
        }
    }

    public boolean canSeeTag() {
        return !aprilTag.getDetections().isEmpty();
    }

    public void stop() {
        visionPortal.close();
    }
}
