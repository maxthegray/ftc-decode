package org.firstinspires.ftc.teamcode.commandbased.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbased.subsystems.DriveSubsystem;

public class GoToOriginCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final boolean maintainHeading;

    public GoToOriginCommand(DriveSubsystem driveSubsystem) {
        this(driveSubsystem, true);
    }

    public GoToOriginCommand(DriveSubsystem driveSubsystem, boolean maintainHeading) {
        this.driveSubsystem = driveSubsystem;
        this.maintainHeading = maintainHeading;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        if (maintainHeading) {
            driveSubsystem.goToOrigin();
        } else {
            driveSubsystem.goToOrigin(0);
        }
    }

    @Override
    public boolean isFinished() {
        return !driveSubsystem.isFollowingPath();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            driveSubsystem.cancelPath();
        }
        driveSubsystem.stop();
    }
}