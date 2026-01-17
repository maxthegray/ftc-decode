package org.firstinspires.ftc.teamcode.commandbased.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.commandbased.subsystems.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.DriveSubsystem;

public class TeleOpDriveCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final AprilTagSubsystem aprilTagSubsystem;
    private final GamepadEx gamepad;

    private boolean autoAlignEnabled = false;

    private static final double ALIGN_POWER = 0.2;
    private static final double BEARING_TOLERANCE = 1;

    public TeleOpDriveCommand(DriveSubsystem driveSubsystem, AprilTagSubsystem aprilTagSubsystem, GamepadEx gamepad) {
        this.driveSubsystem = driveSubsystem;
        this.aprilTagSubsystem = aprilTagSubsystem;
        this.gamepad = gamepad;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double forwardSpeed = gamepad.getLeftY();
        double strafeSpeed = -gamepad.getLeftX();
        double rotationSpeed = -gamepad.getRightX() / 2;

        // Auto-align overrides rotation when enabled
        if (autoAlignEnabled && aprilTagSubsystem.hasBasketTag()) {
            double bearing = aprilTagSubsystem.getTagBearing();
            if (Math.abs(bearing) > BEARING_TOLERANCE) {
                rotationSpeed = ALIGN_POWER * bearing/30;
            } else {
                rotationSpeed = 0;
            }
        }

        driveSubsystem.drive(forwardSpeed, strafeSpeed, rotationSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    public void toggleAutoAlign() {
        autoAlignEnabled = !autoAlignEnabled;
    }

    public boolean isAutoAlignEnabled() {
        return autoAlignEnabled;
    }

    public void setAutoAlignEnabled(boolean enabled) {
        autoAlignEnabled = enabled;
    }
}