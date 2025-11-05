package org.firstinspires.ftc.teamcode.commandbased.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.commandbased.subsystems.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.DriveSubsystem;

public class AprilTagAlignCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final AprilTagSubsystem aprilTagSubsystem;
    private final GamepadEx gamepad;

    private static final double ROTATION_POWER = 0.085;
    private static final double TOLERANCE = 1.0; // degrees

    public AprilTagAlignCommand(DriveSubsystem driveSubsystem,
                                AprilTagSubsystem aprilTagSubsystem,
                                GamepadEx gamepad) {
        this.driveSubsystem = driveSubsystem;
        this.aprilTagSubsystem = aprilTagSubsystem;
        this.gamepad = gamepad;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // Check if basket tag is visible before starting
        if (!aprilTagSubsystem.hasBasketTag()) {
            cancel();
        }
    }

    @Override
    public void execute() {
        // Check if we still have a tag
        if (!aprilTagSubsystem.hasBasketTag()) {
            cancel();
            return;
        }

        // Get manual drive inputs from gamepad
        double strafeSpeed = -gamepad.getLeftX();
        double forwardSpeed = -gamepad.getLeftY();

        // Get bearing to tag
        double bearing = aprilTagSubsystem.getTagBearing();

        // Calculate rotation based on bearing
        double rotationSpeed = 0;
        if (bearing > TOLERANCE) {
            rotationSpeed = ROTATION_POWER;
        } else if (bearing < -TOLERANCE) {
            rotationSpeed = -ROTATION_POWER;
        }

        // Drive with manual strafe/forward, but auto rotation
        driveSubsystem.drive(strafeSpeed, forwardSpeed, rotationSpeed);

        // Add telemetry
        aprilTagSubsystem.addTelemetry();
    }

    @Override
    public boolean isFinished() {
        // Command never finishes on its own - must be cancelled with B button
        // or when tag is lost
        return !aprilTagSubsystem.hasBasketTag();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }
}