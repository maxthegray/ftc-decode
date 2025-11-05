package org.firstinspires.ftc.teamcode.commandbased.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem extends SubsystemBase {

    private final MecanumDrive drive;

    public DriveSubsystem(HardwareMap hardwareMap) {
        // Initialize motors
        Motor frontLeft = new Motor(hardwareMap, "frontLeft");
        Motor frontRight = new Motor(hardwareMap, "frontRight");
        Motor backLeft = new Motor(hardwareMap, "backLeft");
        Motor backRight = new Motor(hardwareMap, "backRight");

        // Create mecanum drive
        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
    }

    public void drive(double strafeSpeed, double forwardSpeed, double rotationSpeed) {
        drive.driveRobotCentric(strafeSpeed, forwardSpeed, rotationSpeed);
    }

    public void stop() {
        drive.stop();
    }
}

