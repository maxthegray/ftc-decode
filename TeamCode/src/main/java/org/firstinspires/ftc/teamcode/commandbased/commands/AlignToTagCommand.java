package org.firstinspires.ftc.teamcode.commandbased.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.commandbased.subsystems.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.DriveSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


//RED BASKET 24
public class AlignToTagCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final AprilTagSubsystem aprilTagSubsystem;
    private final GamepadEx gamepad;

    private static final double ROTATION_POWER = 0.085;
    private static final double BEARING_TOLERANCE = 1.0; // degrees

    public AlignToTagCommand(DriveSubsystem driveSubsystem,
                             AprilTagSubsystem aprilTagSubsystem,
                             GamepadEx gamepad) {
        this.driveSubsystem = driveSubsystem;
        this.aprilTagSubsystem = aprilTagSubsystem;
        this.gamepad = gamepad;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double forwardSpeed = -gamepad.getLeftY();
        double strafeSpeed = -gamepad.getLeftX();

        // apriltag bearing auto-rotation
        double rotationSpeed = calculateAlignmentRotation();

        // manual movement auto rotation
        driveSubsystem.drive(forwardSpeed, strafeSpeed, rotationSpeed);
    }

    private double calculateAlignmentRotation() {
        AprilTagDetection basketTag = aprilTagSubsystem.getBasketDetection();

        if (basketTag == null) {
            return 0; //no tag, no rotation
        }

        double bearing = basketTag.ftcPose.bearing;

        if (bearing > BEARING_TOLERANCE) {
            return ROTATION_POWER;
        } else if (bearing < -BEARING_TOLERANCE) {
            return -ROTATION_POWER;
        }

        return 0;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}