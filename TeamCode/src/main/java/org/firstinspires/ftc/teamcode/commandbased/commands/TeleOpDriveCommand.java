package org.firstinspires.ftc.teamcode.commandbased.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commandbased.subsystems.DriveSubsystem;

public class TeleOpDriveCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private GamepadEx gamepad;

    public TeleOpDriveCommand(DriveSubsystem driveSubsystem, GamepadEx gamepad) {
        this.driveSubsystem = driveSubsystem;
        this.gamepad = gamepad;


        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.initializeTeleOpDrive();
    }

    @Override
    public void execute() {
        double forwardSpeed = gamepad.getLeftY();
        double strafeSpeed = -gamepad.getLeftX();
        double rotationSpeed = -gamepad.getRightX()/2;
        // drive w pedro
        driveSubsystem.drive(forwardSpeed, strafeSpeed, rotationSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }
}