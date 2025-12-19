package org.firstinspires.ftc.teamcode.commandbased;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbased.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.DriveSubsystem;

@TeleOp(name = "Drive (commandbased, with pedro)")
public class DriveOpMode extends CommandOpMode {

    private DriveSubsystem driveSubsystem;
    private TeleOpDriveCommand driveCommand;

    @Override
    public void initialize() {
        driveSubsystem = new DriveSubsystem(hardwareMap);

        driveCommand = new TeleOpDriveCommand(driveSubsystem, gamepad1);

        driveSubsystem.setDefaultCommand(driveCommand);

    }

    @Override
    public void run() {

        com.pedropathing.geometry.Pose currentPose = driveSubsystem.getPose();
        telemetry.addData("X Position", "%.2f inches", currentPose.getX());
        telemetry.addData("Y Position", "%.2f inches", currentPose.getY());
        telemetry.addData("Heading", "%.2f degrees", Math.toDegrees(currentPose.getHeading()));
        telemetry.update();

        super.run();
    }
}