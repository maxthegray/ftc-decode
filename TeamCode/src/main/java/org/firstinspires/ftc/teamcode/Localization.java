package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Localization {

    private AprilTagProcessor aprilTagProcessor;
    private double x;   // robot X position (inches or mm, depending on setup)
    private double y;   // robot Y position
    private double heading; // robot heading (radians)

    public Localization(AprilTagProcessor aprilTagProcessor) {
        this.aprilTagProcessor = aprilTagProcessor;
        this.x = 0;
        this.y = 0;
        this.heading = 0;
    }

    public void update() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        if (detections.size() > 0) {
            // Take the first detection for now
            AprilTagDetection tag = detections.get(0);

            // get position from tag data
            x = tag.ftcPose.x;
            y = tag.ftcPose.y;
            heading = tag.ftcPose.yaw; // yaw in DEGREES
        }
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Loc X", x);
        telemetry.addData("Loc Y", y);
        telemetry.addData("Heading", heading);
    }
}
