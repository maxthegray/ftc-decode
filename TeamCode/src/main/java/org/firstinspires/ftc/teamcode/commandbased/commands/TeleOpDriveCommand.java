package org.firstinspires.ftc.teamcode.commandbased.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commandbased.subsystems.DriveSubsystem;

public class TeleOpDriveCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private Gamepad gamepad;

    public TeleOpDriveCommand(DriveSubsystem driveSubsystem, Gamepad gamepad) {
        this.driveSubsystem = driveSubsystem;
        this.gamepad = gamepad;

        driveSubsystem.initializeTeleOpDrive();

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double forwardSpeed = -gamepad.left_stick_y;
        double strafeSpeed = -gamepad.left_stick_x;
        double rotationSpeed = -gamepad.right_stick_y;

        // drive w pedro
        driveSubsystem.drive(forwardSpeed, strafeSpeed, rotationSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }
}