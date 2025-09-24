package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class Localization {
    public int colorID;
    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

    private double x = 0;       // inches
    private double y = 0;       // inches
    private double heading = 0; // degrees

    public Localization(HardwareMap hardwareMap) {
        // aprilTag processor build
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawTagOutline(true)
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        // Build VisionPortal with camera settings idk if this works yet
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "hsc")); // your webcam name
        builder.setCameraResolution(new Size(1280, 800));
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.enableLiveView(true);
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
    }

    public void update() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == 23 || detection.id == 22 || detection.id == 21) {
                colorID = detection.id;
            } else {
                x = detection.ftcPose.x;       // right
                y = detection.ftcPose.y;       // forward
                heading = detection.ftcPose.yaw; // degrees
            }

        }
    }

    public int getID() {
        return aprilTag.getDetections().get(0).id;
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Loc X (in)", x);
        telemetry.addData("Loc Y (in)", y);
        telemetry.addData("Heading (deg)", heading);
        telemetry.addData("Visible Tags", aprilTag.getDetections().size());
    }

    public void stop() {
        visionPortal.close();
    }
}
