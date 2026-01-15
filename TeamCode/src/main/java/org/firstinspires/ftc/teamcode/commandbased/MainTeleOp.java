package org.firstinspires.ftc.teamcode.commandbased;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbased.commands.GoToOriginCommand;
import org.firstinspires.ftc.teamcode.commandbased.commands.IndexCommand;
import org.firstinspires.ftc.teamcode.commandbased.commands.KickCommand;
import org.firstinspires.ftc.teamcode.commandbased.commands.RotateCarouselCommand;
import org.firstinspires.ftc.teamcode.commandbased.commands.ShootSequenceCommand;
import org.firstinspires.ftc.teamcode.commandbased.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.CarouselSubsystem.BallColor;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.ShooterSubsystem;

@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends CommandOpMode {

    private DriveSubsystem driveSubsystem;
    private CarouselSubsystem carouselSubsystem;
    private ShooterSubsystem shooterSubsystem;

    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    // Auto-indexing
    private boolean ballWasInIntake = false;
    private enum IntakeState { COLLECTING, FULL }
    private IntakeState intakeState = IntakeState.COLLECTING;

    // Default shooting order CHANGE WITH APRILTAG LATER
    private static final BallColor[] SHOOT_ORDER = { BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE };

    @Override
    public void initialize() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        driveSubsystem = new DriveSubsystem(hardwareMap);
        carouselSubsystem = new CarouselSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);

        //GAMEPAD 1

        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new GoToOriginCommand(driveSubsystem));

        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> driveSubsystem.cancelPath(), driveSubsystem));

        // GAMEPAD 2

        operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> carouselSubsystem.stopIntake(), carouselSubsystem));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new IndexCommand(carouselSubsystem));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new KickCommand(carouselSubsystem));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ShootSequenceCommand(carouselSubsystem, SHOOT_ORDER));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new RotateCarouselCommand(carouselSubsystem, RotateCarouselCommand.Direction.LEFT));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new RotateCarouselCommand(carouselSubsystem, RotateCarouselCommand.Direction.RIGHT));

        // Register subsystems
        register(driveSubsystem, carouselSubsystem, shooterSubsystem);

        driveSubsystem.setDefaultCommand(new TeleOpDriveCommand(driveSubsystem, driverGamepad));

    }

    @Override
    public void run() {
        super.run();

        handleTriggers();

        updateTelemetry();
    }

    private void handleTriggers() {
        double rightTrigger = operatorGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        double leftTrigger = operatorGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

        //intake state
        if (carouselSubsystem.isFull()) {
            intakeState = IntakeState.FULL;
            ballWasInIntake = true; // prevent false triggers TEST
        } else if (intakeState == IntakeState.FULL) {
            // Just became not-full, now switch to collecting
            intakeState = IntakeState.COLLECTING;
            ballWasInIntake = true; // Ignore the ball already sitting at intake
        }

        if (rightTrigger > 0.1) {
            carouselSubsystem.runIntake();

            // Only auto switch in COLLECTING state
            if (intakeState == IntakeState.COLLECTING && carouselSubsystem.isSettled()) {
                BallColor intakeContents = carouselSubsystem.getIntakeContents();
                boolean hasBall = (intakeContents == BallColor.GREEN || intakeContents == BallColor.PURPLE);

                if (hasBall && !ballWasInIntake) {
                    carouselSubsystem.rotateEmptyToIntake();
                }
                ballWasInIntake = hasBall;
            }
        }
        else if (leftTrigger > 0.1) {
            carouselSubsystem.reverseIntake();
            ballWasInIntake = false;
        }
        else {
            ballWasInIntake = false;
        }

    }


    private void updateTelemetry() {
        telemetry.addData("Current Ticks", carouselSubsystem.getCurrentTicks());
        telemetry.addData("Target Ticks", carouselSubsystem.getTargetTicks());
        telemetry.addData("Settled", carouselSubsystem.isSettled());
        telemetry.addData("Intake State", intakeState);

        BallColor[] positions = carouselSubsystem.getAllPositions();
        telemetry.addData("INTAKE", positions[CarouselSubsystem.POS_INTAKE]);
        telemetry.addData("BACK_LEFT", positions[CarouselSubsystem.POS_BACK_LEFT]);
        telemetry.addData("BACK_RIGHT", positions[CarouselSubsystem.POS_BACK_RIGHT]);
        telemetry.addData("getPose", driveSubsystem.getPose());

        telemetry.update();

    }
}