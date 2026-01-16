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
import org.firstinspires.ftc.teamcode.commandbased.subsystems.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.CarouselSubsystem.BallColor;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.ShooterSubsystem;

@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends CommandOpMode {

    private DriveSubsystem driveSubsystem;
    private CarouselSubsystem carouselSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private AprilTagSubsystem aprilTagSubsystem;

    private TeleOpDriveCommand teleOpDriveCommand;

    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    // Auto-indexing
    private boolean ballWasInIntake = false;
    private enum IntakeState { COLLECTING, FULL }
    private IntakeState intakeState = IntakeState.COLLECTING;

    // Shooter velocity (degrees/second)
    private double shooterVelocity = 0.0;
    private static final double VELOCITY_INCREMENT_SMALL = 1.0;   // ±1 deg/s (fine)
    private static final double VELOCITY_INCREMENT_LARGE = 20.0;   // ±5 deg/s (coarse)

    // Shooter mutex - shooter only runs when robot is stationary and intake is off
    private boolean intakeRunning = false;
    private boolean robotMoving = false;

    private static final BallColor[] SHOOT_ORDER = { BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE };

    @Override
    public void initialize() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        // Initialize subsystems
        driveSubsystem = new DriveSubsystem(hardwareMap);
        carouselSubsystem = new CarouselSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        aprilTagSubsystem = new AprilTagSubsystem(hardwareMap, telemetry);

        // Create drive command with AprilTag support
        teleOpDriveCommand = new TeleOpDriveCommand(driveSubsystem, aprilTagSubsystem, driverGamepad);

        // ==================== DRIVER CONTROLS (Gamepad 1) ====================

        // Left bumper - Toggle auto-align
        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> teleOpDriveCommand.toggleAutoAlign()));

        // Right bumper - Go to origin
        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new GoToOriginCommand(driveSubsystem));

        // Back - Cancel path
        driverGamepad.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(() -> driveSubsystem.cancelPath(), driveSubsystem));

        // Square - Show lights for 3 seconds
        operatorGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> carouselSubsystem.showLights()));



        // Bumpers - Shooter velocity (±50 deg/s)
        operatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> adjustShooterVelocity(-VELOCITY_INCREMENT_LARGE)));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> adjustShooterVelocity(VELOCITY_INCREMENT_LARGE)));

        // D-pad up/down - Shooter velocity fine adjust (±10 deg/s)
        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> adjustShooterVelocity(VELOCITY_INCREMENT_SMALL)));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> adjustShooterVelocity(-VELOCITY_INCREMENT_SMALL)));
// THIS WILL BE FOR FINAL
//        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(new InstantCommand(() -> shooterSubsystem.setAdjustedVelocity(aprilTagSubsystem.getDistanceToTag())));

        // X - Shooter off
        operatorGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> {
                    shooterSubsystem.stop();
                }));

        // D-pad left/right - Manual carousel rotation
        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new RotateCarouselCommand(carouselSubsystem, RotateCarouselCommand.Direction.LEFT));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new RotateCarouselCommand(carouselSubsystem, RotateCarouselCommand.Direction.RIGHT));

        // A - Index
        operatorGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new IndexCommand(carouselSubsystem));

        // B - Manual kick
        operatorGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new KickCommand(carouselSubsystem));

        // Y - Shoot sequence
        operatorGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ShootSequenceCommand(carouselSubsystem, SHOOT_ORDER));

        // Register subsystems
        register(driveSubsystem, carouselSubsystem, shooterSubsystem, aprilTagSubsystem);

        driveSubsystem.setDefaultCommand(teleOpDriveCommand);
    }

    @Override
    public void run() {
        super.run();
        updateMutexState();
        handleTriggers();
        handleShooter();
        updateTelemetry();
    }

    private void updateMutexState() {
        // Check if robot is moving (joystick input or following a path)
        double forward = Math.abs(driverGamepad.getLeftY());
        double strafe = Math.abs(driverGamepad.getLeftX());
        double rotate = Math.abs(driverGamepad.getRightX());
        robotMoving = (forward > 0.1 || strafe > 0.1 || rotate > 0.1 || driveSubsystem.isFollowingPath());
    }

    private void adjustShooterVelocity(double delta) {
        shooterVelocity = Math.max(0, shooterVelocity + delta);
    }

    private void handleShooter() {
        // Shooter only runs when robot is stationary and intake is off
        boolean canRunShooter = !robotMoving && !intakeRunning;

        if (canRunShooter && shooterVelocity > 0) {
            shooterSubsystem.setTargetVelocity(shooterVelocity);
        } else {
            shooterSubsystem.stop();
            shooterVelocity = 0;
        }
    }

    private void handleTriggers() {
        double rightTrigger = driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        double leftTrigger = driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

        // Update intake state
        if (carouselSubsystem.isFull()) {
            intakeState = IntakeState.FULL;
            ballWasInIntake = true;
        } else if (intakeState == IntakeState.FULL) {
            intakeState = IntakeState.COLLECTING;
            ballWasInIntake = true;
        }

        // Right trigger - run intake
        if (rightTrigger > 0.1) {
            intakeRunning = true;
            carouselSubsystem.runIntake();

            // Auto-index in COLLECTING state
            if (intakeState == IntakeState.COLLECTING && carouselSubsystem.isSettled()) {
                BallColor intakeContents = carouselSubsystem.getIntakeContents();
                boolean hasBall = (intakeContents == BallColor.GREEN || intakeContents == BallColor.PURPLE);

                if (hasBall && !ballWasInIntake) {
                    carouselSubsystem.rotateEmptyToIntake();
                }
                ballWasInIntake = hasBall;
            }
        }
        // Left trigger - reverse intake
        else if (leftTrigger > 0.1) {
            intakeRunning = true;
            carouselSubsystem.reverseIntake();
            ballWasInIntake = false;
        }
        // No trigger - stop intake
        else {
            intakeRunning = false;
            carouselSubsystem.stopIntake();
            ballWasInIntake = false;
        }


    }

    private void updateTelemetry() {
        // Shooter info
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Target Velocity", "%.0f deg/s", shooterVelocity);
        telemetry.addData("Current Velocity", "%.0f deg/s", shooterSubsystem.getCurrentVelocity());
        telemetry.addData("Can Run", (!robotMoving && !intakeRunning) ? "YES" : "NO (moving/intake)");
        telemetry.addData("Auto-Align", teleOpDriveCommand.isAutoAlignEnabled() ? "ON" : "OFF");

        // AprilTag info
        telemetry.addLine("=== APRILTAG ===");
        if (aprilTagSubsystem.hasBasketTag()) {
            telemetry.addData("Distance", "%.1f in", aprilTagSubsystem.getTagRange());
            telemetry.addData("Bearing", "%.1f°", aprilTagSubsystem.getTagBearing());
        } else {
            telemetry.addData("Tag", "NOT VISIBLE");
        }

        // Carousel info
        telemetry.addLine("=== CAROUSEL ===");
        telemetry.addData("Ball Count", carouselSubsystem.getBallCount() + "/3");
        telemetry.addData("Intake State", intakeState);

        BallColor[] positions = carouselSubsystem.getAllPositions();
        telemetry.addData("INTAKE", positions[CarouselSubsystem.POS_INTAKE]);
        telemetry.addData("BACK_LEFT", positions[CarouselSubsystem.POS_BACK_LEFT]);
        telemetry.addData("BACK_RIGHT", positions[CarouselSubsystem.POS_BACK_RIGHT]);

        // Drive info
        telemetry.addLine("=== DRIVE ===");
        telemetry.addData("Pose", driveSubsystem.getPose());
        telemetry.addData("Following Path", driveSubsystem.isFollowingPath());

        telemetry.update();
    }
}