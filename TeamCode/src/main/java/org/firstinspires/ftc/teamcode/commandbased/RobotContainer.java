package org.firstinspires.ftc.teamcode.commandbased;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commandbased.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.DriveSubsystem;


@TeleOp(name = "Mecanum Drive with AprilTag Align")
public class RobotContainer extends CommandOpMode {

    private DriveSubsystem driveSubsystem;
    private AprilTagSubsystem aprilTagSubsystem;
    private GamepadEx driverGamepad;

    @Override
    public void initialize() {
        // Initialize subsystems
        driveSubsystem = new DriveSubsystem(hardwareMap);
        aprilTagSubsystem = new AprilTagSubsystem(hardwareMap, telemetry);

        // Initialize gamepad
        driverGamepad = new GamepadEx(gamepad1);

        // Set default command (normal driving)
        driveSubsystem.setDefaultCommand(
                new DriveCommand(driveSubsystem, driverGamepad)
        );

        // Configure button bindings
        configureButtonBindings();

        telemetry.addLine("Robot Initialized!");
        telemetry.addLine("Left stick = Drive");
        telemetry.addLine("Right stick = Rotate");
        telemetry.addLine("A button = Align to AprilTag");
        telemetry.addLine("B button = Cancel alignment");
        telemetry.update();
    }

    private void configureButtonBindings() {
        // A button - Align to AprilTag (ID 20 or 24)
        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenPressed(new org.firstinspires.ftc.teamcode.commandbased.commands.AprilTagAlignCommand(driveSubsystem, aprilTagSubsystem, driverGamepad));

        // B button - Cancel and return to manual drive
        new GamepadButton(driverGamepad, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {
                    if (driveSubsystem.getCurrentCommand() instanceof org.firstinspires.ftc.teamcode.commandbased.commands.AprilTagAlignCommand) {
                        driveSubsystem.getCurrentCommand().cancel();
                    }
                }));
    }
}