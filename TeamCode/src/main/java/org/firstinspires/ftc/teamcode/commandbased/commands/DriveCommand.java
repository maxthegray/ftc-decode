package org.firstinspires.ftc.teamcode.commandbased.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.commandbased.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final GamepadEx gamepad;

    public DriveCommand(DriveSubsystem driveSubsystem, GamepadEx gamepad) {
        this.driveSubsystem = driveSubsystem;
        this.gamepad = gamepad;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        // Get gamepad inputs
        double strafeSpeed = -gamepad.getLeftX();
        double forwardSpeed = -gamepad.getLeftY();
        double rotationSpeed = -gamepad.getRightX();

        // Drive the robot
        driveSubsystem.drive(strafeSpeed, forwardSpeed, rotationSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }
}
