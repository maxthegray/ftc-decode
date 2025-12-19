package org.firstinspires.ftc.teamcode.commandbased;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbased.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.commandbased.commands.GoToPositionCommand;
import org.firstinspires.ftc.teamcode.commandbased.commands.KickCommand;
import org.firstinspires.ftc.teamcode.commandbased.commands.ShootSequenceCommand;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.CarouselSubsystem.BallColor;

@TeleOp(name = "Main TeleOp")
public class DriveAndShootOpMode extends CommandOpMode {

    private DriveSubsystem drive;
    private CarouselSubsystem carousel;
    private GamepadEx controller;

    @Override
    public void initialize() {
        // Subsystems
        drive = new DriveSubsystem(hardwareMap);
        carousel = new CarouselSubsystem(hardwareMap);
        controller = new GamepadEx(gamepad1);

        // Set initial slot contents (until color sensors are added)
        carousel.setAllSlots(BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE);

        // Default command: always drive with gamepad
        drive.setDefaultCommand(new TeleOpDriveCommand(drive, gamepad1));

        // Carousel position control (runs alongside driving!)
        controller.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new GoToPositionCommand(carousel, 0));

        controller.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new GoToPositionCommand(carousel, 1));

        controller.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new GoToPositionCommand(carousel, 2));

        // Kick
        controller.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new KickCommand(carousel));

        // Full shoot sequence (also runs alongside driving!)
        controller.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ShootSequenceCommand(
                        carousel,
                        BallColor.GREEN,
                        BallColor.PURPLE,
                        BallColor.PURPLE
                ));
    }

    @Override
    public void run() {
        super.run(); // Runs command scheduler

        // Telemetry
        telemetry.addData("-- DRIVE --", "");
        telemetry.addData("Pose", drive.getPose().toString());

        telemetry.addData("-- CAROUSEL --", "");
        telemetry.addData("Position", carousel.getCurrentPosition());
        telemetry.addData("Target", carousel.getTargetPosition());
        telemetry.addData("Settled", carousel.isSettled());

        telemetry.addData("-- SLOTS --", "");
        telemetry.addData("Slot 0", carousel.getSlotContents(0));
        telemetry.addData("Slot 1", carousel.getSlotContents(1));
        telemetry.addData("Slot 2", carousel.getSlotContents(2));

        telemetry.update();
    }
}
