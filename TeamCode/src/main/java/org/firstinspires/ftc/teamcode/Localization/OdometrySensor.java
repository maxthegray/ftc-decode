
package org.firstinspires.ftc.teamcode.Localization;

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

public class OdometrySensor {

    private SparkFunOTOS myOtos;
    private double startx;
    private double starty;
    private double startheading;
    private Telemetry telemetry;

    // Odometry stuff -------------------
    public OdometrySensor(Telemetry telemetryy, HardwareMap hardwareMap, double sx, double sy, double sh) {
        myOtos = hardwareMap.get(SparkFunOTOS.class, "odometrySensor");
        startx = sx;
        starty = sy;
        startheading = sh;
        telemetry = telemetryy;
        configureOtos();

    }

    public void configureOtos() {
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-7, -4, 0);
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

    public void updateOdo(double x, double y, double heading) {
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(x, y, heading);
        myOtos.setPosition(currentPosition);
    }

    public void addTelemetry() {

        telemetry.addData("Odo X (in)", getOdoX());
        telemetry.addData("Odo Y (in)", getOdoY());
        telemetry.addData("Odo Heading (deg)", getOdoHeading());

    }
}
