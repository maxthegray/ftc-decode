package org.firstinspires.ftc.teamcode.commandbased;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbased.commands.AlignToTagCommand;
import org.firstinspires.ftc.teamcode.commandbased.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commandbased.commands.FeedBallCommand;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.ShooterSubsystem;

@TeleOp(name = "Shooter State Machine")
public class ShooterStateMachineOpMode extends CommandOpMode {

    // State machine enum
    private enum RobotState {
        DRIVING,
        SHOOTING
    }

    private RobotState currentState = RobotState.DRIVING;
    private RobotState previousState = RobotState.DRIVING;

    // Subsystems
    private DriveSubsystem driveSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private AprilTagSubsystem aprilTagSubsystem;

    // Commands
    private DriveCommand driveCommand;
    private AlignToTagCommand alignToTagCommand;

    // Gamepads
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    // Button tracking
    private boolean lastStateToggle = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastAutoToggle = false;

    // Auto shooter control
    private boolean autoShooterPower = true; // Start with auto mode enabled

    @Override
    public void initialize() {
        // Initialize subsystems
        driveSubsystem = new DriveSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        aprilTagSubsystem = new AprilTagSubsystem(hardwareMap, telemetry);

        // Initialize gamepads
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        // Create commands
        driveCommand = new DriveCommand(driveSubsystem, driverGamepad);
        alignToTagCommand = new AlignToTagCommand(driveSubsystem, aprilTagSubsystem, driverGamepad);

        // Set initial state to DRIVING
        setDrivingState();

        // Configure button bindings for operator (shooter control)
        configureOperatorButtons();

        // Configure button bindings for driver (path following)
        configureDriverButtons();

        telemetry.update();
    }

    @Override
    public void run() {
        // Handle state transitions
        handleStateToggle();

        // Handle shooter power adjustments
        handleShooterPowerControl();

        // Fuse AprilTag vision with OTOS odometry for improved localization
        updateLocalizationWithVision();

        // Update telemetry
        updateTelemetry();

        // Run the command scheduler (this handles command execution)
        super.run();
    }

    /**
     * Update robot localization using AprilTag vision when available.
     * This is PASSIVE normally, but ACTIVE on first detection to initialize position.
     * - First detection: Uses any reliable tag to set initial position
     * - Subsequent detections: Only updates when tag is very reliable
     * OTOS continues to run normally between updates.
     */
    private void updateLocalizationWithVision() {
        if (!aprilTagSubsystem.hasBasketTag()) {
            return;
        }

        // If pose not initialized yet, be more lenient with first detection
        boolean isFirstDetection = !driveSubsystem.isPoseInitialized();
        boolean shouldUpdate;

        if (isFirstDetection) {
            // First time seeing tag - just need it to be detected
            // We'll use any detection to initialize position
            shouldUpdate = true;
        } else {
            // Already initialized - be more selective for updates
            shouldUpdate = true;
        }

        if (shouldUpdate) {
            Pose visionPose = aprilTagSubsystem.getRobotPoseFromAprilTag();

            if (visionPose != null) {
                // Trust AprilTag 100% - it's more accurate than OTOS drift
                driveSubsystem.updatePoseFromVision(visionPose);
            }
        }
    }

    private void handleStateToggle() {
        boolean currentToggle = gamepad1.a;

        if (currentToggle && !lastStateToggle) {
            // Toggle state
            if (currentState == RobotState.DRIVING) {
                setShootingState();
            } else {
                setDrivingState();
            }
        }

        lastStateToggle = currentToggle;
    }

    private void setDrivingState() {
        currentState = RobotState.DRIVING;

        // Cancel any existing commands and set drive command as default
        CommandScheduler.getInstance().cancelAll();
        driveSubsystem.setDefaultCommand(driveCommand);

        // Stop shooter
        shooterSubsystem.setTargetPower(0);
    }

    private void setShootingState() {
        currentState = RobotState.SHOOTING;

        // Cancel existing commands and set align command as default
        CommandScheduler.getInstance().cancelAll();
        driveSubsystem.setDefaultCommand(alignToTagCommand);

        // Set shooter to default power
        shooterSubsystem.setTargetPower(0.5);
    }

    private void handleShooterPowerControl() {
        boolean currentDpadUp = gamepad2.dpad_up;
        boolean currentDpadDown = gamepad2.dpad_down;

        if (currentDpadUp && !lastDpadUp) {
            shooterSubsystem.increasePower();
        }

        if (currentDpadDown && !lastDpadDown) {
            shooterSubsystem.decreasePower();
        }

        lastDpadUp = currentDpadUp;
        lastDpadDown = currentDpadDown;
    }

    private void configureOperatorButtons() {
        // X button - Feed ball when shooter is ready
        new GamepadButton(operatorGamepad, GamepadKeys.Button.X)
                .whenPressed(() -> {
                    if (isShooterReady()) {
                        CommandScheduler.getInstance().schedule(new FeedBallCommand(telemetry));
                    } else {
                        telemetry.addLine("Shooter not ready!");
                        telemetry.update();
                    }
                });
    }

    private void configureDriverButtons() {
        // Y button - Go to origin (0, 0)
        new GamepadButton(driverGamepad, GamepadKeys.Button.Y)
                .whenPressed(() -> {
                    driveSubsystem.goToOrigin();
                    telemetry.addLine("Going to origin...");
                    telemetry.update();
                });

        // B button (hold) - Cancel path following, return to manual control
        new GamepadButton(driverGamepad, GamepadKeys.Button.B)
                .whenPressed(() -> {
                    if (driveSubsystem.isFollowingPath()) {
                        driveSubsystem.cancelPath();
                        telemetry.addLine("Path cancelled - Manual control");
                        telemetry.update();
                    }
                });
    }

    private boolean isShooterReady() {
        // Shooter is ready if:
        // 1. Robot is in SHOOTING state
        // 2. Shooter motor is at target speed
        // 3. Robot is aligned to AprilTag

        if (currentState != RobotState.SHOOTING) {
            return false;
        }

        boolean motorAtSpeed = shooterSubsystem.isAtTargetSpeed();
        boolean robotAligned = isRobotAligned();

        return motorAtSpeed && robotAligned;
    }

    private boolean isRobotAligned() {
        double bearing = aprilTagSubsystem.getTagBearing();
        return aprilTagSubsystem.hasBasketTag() && Math.abs(bearing) <= 1.0;
    }

    private void updateTelemetry() {
        telemetry.addData("STATE", currentState.toString());
        telemetry.addLine();

        // Pose initialization status
        if (!driveSubsystem.isPoseInitialized()) {
            telemetry.addData("POSITION", "NOT SEEN TAG");
        } else {
            telemetry.addData("POSITION", "SEEN");
        }

        // Localization info
        Pose currentPose = driveSubsystem.getPose();
        telemetry.addData("Robot X", "%.2f inches", currentPose.getX());
        telemetry.addData("Robot Y", "%.2f inches", currentPose.getY());
        telemetry.addData("Robot Heading", "%.2f degrees", Math.toDegrees(currentPose.getHeading()));

        // Path following status
        if (driveSubsystem.isFollowingPath()) {
            telemetry.addData("Path Following", "ACTIVE");
        }

        // Show if vision is updating localization (PASSIVE)
        if (aprilTagSubsystem.hasBasketTag()) {
            if (!driveSubsystem.isPoseInitialized()) {
                telemetry.addData("Vision Update", "INITIALIZING");
            } else if (aprilTagSubsystem.canSeeRedTag()){
                telemetry.addData("Vision Update", "ACTIVE");
            }
        } else {
            telemetry.addData("Vision Update", "OTOS Only");
        }
        telemetry.addLine();

        // Shooter info
        telemetry.addData("Target Power", "%.2f (%.0f%%)",
                shooterSubsystem.getTargetPower(),
                shooterSubsystem.getTargetPower() * 100);
        telemetry.addData("Current Power", "%.2f (%.0f%%)",
                shooterSubsystem.getCurrentPower(),
                shooterSubsystem.getCurrentPower() * 100);
        telemetry.addData("Motor At Speed", shooterSubsystem.isAtTargetSpeed() ? "YES ✓" : "NO");
        telemetry.addData("Robot Aligned", isRobotAligned() ? "YES ✓" : "NO");

        // Show distance when aligned
        if (isRobotAligned() && aprilTagSubsystem.hasBasketTag()) {
            telemetry.addData("Distance to Target", "%.2f inches", aprilTagSubsystem.getDistanceToTag());
        }

        telemetry.addData("SHOOTER READY", isShooterReady() ? "YES" : "NO");
        telemetry.addLine();

        // AprilTag info
        aprilTagSubsystem.addTelemetry();

        telemetry.update();
    }
}