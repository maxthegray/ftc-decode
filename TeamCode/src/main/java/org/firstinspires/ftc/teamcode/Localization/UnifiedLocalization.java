
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

public class UnifiedLocalization {

    OdometrySensor odo;
    LocalizationCamera cam;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    private double absX;
    private double absY;
    private double absH;


    public UnifiedLocalization(Telemetry telemetryy, HardwareMap hardwareMap) {
       odo = new OdometrySensor(telemetryy, hardwareMap, cam.getX(), cam.getY(), cam.getHeading());
       cam = new LocalizationCamera(telemetryy, hardwareMap);
       this.hardwareMap = hardwareMap;
       this.telemetry = telemetryy;
    }

    public void AbsoluteXYH() {
        if (cam.tagInView()) {
            absX = (cam.getX() * 0.8) + (odo.getOdoX() * 0.2);
            absY = (cam.getY() * 0.8) + (odo.getOdoY() * 0.2);
            absH = (cam.getHeading() * 0.8) + (odo.getOdoHeading() * 0.2);

            odo.updateOdo(cam.getX(), cam.getY(), cam.getHeading());

        } else {
            absX = odo.getOdoX();
            absY = odo.getOdoY();
            absH = odo.getOdoHeading();
        }
    }

    public void update() {
        cam.updateAprilTag();
        AbsoluteXYH();
    }

    public double getX() { return absX; }
    public double getY() { return absY; }
    public double getHeading() { return absH; }

}
