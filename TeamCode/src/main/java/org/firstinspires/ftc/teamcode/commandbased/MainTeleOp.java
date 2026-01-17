package org.firstinspires.ftc.teamcode.commandbased;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbased.commands.IndexCommand;
import org.firstinspires.ftc.teamcode.commandbased.commands.KickCommand;
import org.firstinspires.ftc.teamcode.commandbased.commands.RotateCarouselCommand;
import org.firstinspires.ftc.teamcode.commandbased.commands.ShootSequenceCommand;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.CarouselSubsystem.BallColor;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends LinearOpMode {

    private Follower follower;
    private CarouselSubsystem carouselSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private AprilTagSubsystem aprilTagSubsystem;

    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    private boolean autoAlignEnabled = false;
    private static final double ALIGN_POWER = 0.15;
    private static final double BEARING_TOLERANCE = 1.0;

    private boolean ballWasInIntake = false;
    private enum IntakeState { COLLECTING, FULL }
    private IntakeState intakeState = IntakeState.COLLECTING;
    private boolean intakeRunning = false;

    private static final BallColor[] SHOOT_ORDER = { BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE };

    @Override
    public void runOpMode() {
        // Step 1: Reset scheduler
        telemetry.addData("Step", "1 - Resetting CommandScheduler");
        telemetry.update();
        CommandScheduler.getInstance().reset();
        sleep(100);

        // Step 2: Gamepads
        telemetry.addData("Step", "2 - Creating GamepadEx");
        telemetry.update();
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);
        sleep(100);

        // Step 3: Drive
        telemetry.addData("Step", "3 - Creating Follower");
        telemetry.update();
        follower = Constants.createFollower(hardwareMap);
        follower.startTeleOpDrive();
        sleep(100);

        // Step 4: Carousel
        telemetry.addData("Step", "4 - Creating CarouselSubsystem");
        telemetry.update();
        carouselSubsystem = new CarouselSubsystem(hardwareMap);
        sleep(100);

        // Step 5: Shooter
        telemetry.addData("Step", "5 - Creating ShooterSubsystem");
        telemetry.update();
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        sleep(100);

        // Step 6: AprilTag (in try-catch)
        telemetry.addData("Step", "6 - Creating AprilTagSubsystem");
        telemetry.update();
        try {
            aprilTagSubsystem = new AprilTagSubsystem(hardwareMap, telemetry);
        } catch (Exception e) {
            telemetry.addData("AprilTag Error", e.getMessage());
            aprilTagSubsystem = null;
        }
        sleep(100);

        // Step 7: Button bindings
        telemetry.addData("Step", "7 - Setting up button bindings");
        telemetry.update();
        setupControls();
        sleep(100);

        // Step 8: Register subsystems
        telemetry.addData("Step", "8 - Registering subsystems");
        telemetry.update();
        if (aprilTagSubsystem != null) {
            CommandScheduler.getInstance().registerSubsystem(carouselSubsystem, shooterSubsystem, aprilTagSubsystem);
        } else {
            CommandScheduler.getInstance().registerSubsystem(carouselSubsystem, shooterSubsystem);
        }
        sleep(100);

        telemetry.addData("Status", "READY - Press Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            driverGamepad.readButtons();
            operatorGamepad.readButtons();

            updateDrive();
            CommandScheduler.getInstance().run();
            handleTriggers();
            handleShooter();
            updateTelemetry();
        }

        CommandScheduler.getInstance().reset();
    }

    private void setupControls() {
        // Driver
        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> autoAlignEnabled = !autoAlignEnabled));

        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    if (aprilTagSubsystem != null) {
                        Pose tagPose = aprilTagSubsystem.getRobotPoseFromAprilTag();
                        if (tagPose != null) {
                            follower.setPose(tagPose);
                        }
                    }
                }));

        driverGamepad.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(() -> follower.breakFollowing()));

        driverGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> carouselSubsystem.showLights()));

        // Operator
        operatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    if (aprilTagSubsystem != null && aprilTagSubsystem.hasBasketTag()) {
                        shooterSubsystem.setAdjustedVelocity(aprilTagSubsystem.getTagRange());
                    }
                }));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> shooterSubsystem.stop()));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new RotateCarouselCommand(carouselSubsystem, RotateCarouselCommand.Direction.LEFT));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new RotateCarouselCommand(carouselSubsystem, RotateCarouselCommand.Direction.RIGHT));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new IndexCommand(carouselSubsystem));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new KickCommand(carouselSubsystem));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ShootSequenceCommand(carouselSubsystem, SHOOT_ORDER));
    }

    private void updateDrive() {
        double forward = driverGamepad.getLeftY();
        double strafe = -driverGamepad.getLeftX();
        double rotate = -driverGamepad.getRightX() / 2.0;

        if (autoAlignEnabled && aprilTagSubsystem != null && aprilTagSubsystem.hasBasketTag()) {
            double bearing = aprilTagSubsystem.getTagBearing();
            if (Math.abs(bearing) > BEARING_TOLERANCE) {
                rotate = ALIGN_POWER * bearing/30;
            } else {
                rotate = 0;
            }
        }

        follower.setTeleOpDrive(forward, strafe, rotate, true);
        follower.update();
    }

    private void handleShooter() {
        if (intakeRunning) {
            shooterSubsystem.stop();
        }
    }

    private void handleTriggers() {
        double rightTrigger = operatorGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        double leftTrigger = operatorGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

        if (carouselSubsystem.isFull()) {
            intakeState = IntakeState.FULL;
            ballWasInIntake = true;
        } else if (intakeState == IntakeState.FULL) {
            intakeState = IntakeState.COLLECTING;
            ballWasInIntake = true;
        }

        if (rightTrigger > 0.1) {
            intakeRunning = true;
            carouselSubsystem.runIntake();

            if (intakeState == IntakeState.COLLECTING && carouselSubsystem.isSettled()) {
                BallColor intakeContents = carouselSubsystem.getIntakeContents();
                boolean hasBall = (intakeContents == BallColor.GREEN || intakeContents == BallColor.PURPLE);

                if (hasBall && !ballWasInIntake) {
                    carouselSubsystem.rotateEmptyToIntake();
                }
                ballWasInIntake = hasBall;
            }
        } else if (leftTrigger > 0.1) {
            intakeRunning = true;
            carouselSubsystem.reverseIntake();
            ballWasInIntake = false;
        } else {
            intakeRunning = false;
            carouselSubsystem.stopIntake();
            ballWasInIntake = false;
        }
    }

    private void updateTelemetry() {
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Target", "%.0f deg/s", shooterSubsystem.getTargetVelocity());
        telemetry.addData("Current", "%.0f deg/s", shooterSubsystem.getCurrentVelocity());

        telemetry.addLine("=== APRILTAG ===");
        telemetry.addData("Auto-Align", autoAlignEnabled ? "ON" : "OFF");
        if (aprilTagSubsystem != null && aprilTagSubsystem.hasBasketTag()) {
            telemetry.addData("Distance", "%.1f in", aprilTagSubsystem.getTagRange());
            telemetry.addData("Bearing", "%.1f°", aprilTagSubsystem.getTagBearing());
        } else {
            telemetry.addData("Tag", aprilTagSubsystem == null ? "DISABLED" : "NOT VISIBLE");
        }

        telemetry.addLine("=== CAROUSEL ===");
        telemetry.addData("Ball Count", carouselSubsystem.getBallCount() + "/3");

        BallColor[] positions = carouselSubsystem.getAllPositions();
        telemetry.addData("INTAKE", positions[CarouselSubsystem.POS_INTAKE]);
        telemetry.addData("BACK_LEFT", positions[CarouselSubsystem.POS_BACK_LEFT]);
        telemetry.addData("BACK_RIGHT", positions[CarouselSubsystem.POS_BACK_RIGHT]);

        telemetry.addLine("=== DRIVE ===");
        telemetry.addData("Pose", follower.getPose());

        telemetry.update();
    }
}