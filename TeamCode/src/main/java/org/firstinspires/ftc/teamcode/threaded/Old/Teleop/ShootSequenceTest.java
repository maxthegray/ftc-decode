package org.firstinspires.ftc.teamcode.threaded.Old.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.threaded.Old.CameraThread;
import org.firstinspires.ftc.teamcode.threaded.Old.CarouselController;
import org.firstinspires.ftc.teamcode.threaded.Old.ControlHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.Old.DriveThread;
import org.firstinspires.ftc.teamcode.threaded.Old.ExpansionHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.Old.MechanismThread;
import org.firstinspires.ftc.teamcode.threaded.Old.SensorState;
import org.firstinspires.ftc.teamcode.threaded.Old.ShootSequence;
import org.firstinspires.ftc.teamcode.threaded.Old.ShooterThread;
@Disabled
@TeleOp(name = "ShootSequence Test", group = "Test")
public class ShootSequenceTest extends LinearOpMode {

    private static final int BASKET_TAG_ID = CameraThread.TAG_RED_BASKET;
    private static final double DEFAULT_VELOCITY = 10.0;

    private MechanismThread mechanismThread;
    private DriveThread driveThread;
    private ShooterThread shooterThread;
    private CameraThread cameraThread;
    private ControlHubI2CThread controlHubI2C;
    private ExpansionHubI2CThread expansionHubI2C;
    private SensorState sensorState;

    private ElapsedTime runtime;

    // Gamepad 1 edge detection
    private boolean prevLBumper1 = false;
    private boolean prevRBumper1 = false;

    // Gamepad 2 edge detection
    private boolean prevA2 = false;
    private boolean prevB2 = false;
    private boolean prevX2 = false;
    private boolean prevY2 = false;
    private boolean prevDpadUp2 = false;
    private boolean prevDpadDown2 = false;
    private boolean prevLTrigger2 = false;
    private boolean prevRTrigger2 = false;

    @Override
    public void runOpMode() {
        runtime = new ElapsedTime();
        sensorState = new SensorState(SensorState.Alliance.BLUE);

        // Initialize threads
        mechanismThread = new MechanismThread(hardwareMap);
        mechanismThread.setSensorState(sensorState);
        driveThread = new DriveThread(sensorState, hardwareMap);
        shooterThread = new ShooterThread(sensorState, hardwareMap);
        cameraThread = new CameraThread(sensorState, hardwareMap, BASKET_TAG_ID);
        controlHubI2C = new ControlHubI2CThread(sensorState, hardwareMap);
        expansionHubI2C = new ExpansionHubI2CThread(sensorState, hardwareMap);

        telemetry.addLine("=== SHOOT SEQUENCE TEST ===");
        telemetry.addLine();
        telemetry.addLine("DRIVER (GP1):");
        telemetry.addLine("  Sticks = Drive");
        telemetry.addLine("  LB = Toggle Auto-Align");
        telemetry.addLine("  RB = Reset Pose from Tag");
        telemetry.addLine("  RT = Intake In");
        telemetry.addLine("  LT = Intake Out");
        telemetry.addLine();
        telemetry.addLine("GUNNER (GP2):");
        telemetry.addLine("  RB = Shooter Spin Up");
        telemetry.addLine("  LB = Shooter Stop");
        telemetry.addLine("  A = Shoot GPP Sequence");
        telemetry.addLine("  B = Shoot PGP Sequence");
        telemetry.addLine("  X = Shoot PPG Sequence");
        telemetry.addLine("  Y = Abort Sequence");
        telemetry.addLine("  LT = Shoot GREEN Single");
        telemetry.addLine("  RT = Shoot PURPLE Single");
        telemetry.addLine("  D-Up = Nudge Carousel +");
        telemetry.addLine("  D-Down = Nudge Carousel -");
        telemetry.update();

        waitForStart();
        runtime.reset();

        mechanismThread.start();
        driveThread.start();
        shooterThread.start();
        cameraThread.start();
        controlHubI2C.start();
        expansionHubI2C.start();

        while (opModeIsActive()) {
            mechanismThread.setBallPositions(sensorState.getAllPositions());

            handleDriverControls();
            handleGunnerControls();

            updateTelemetry();
        }

        // Cleanup
        mechanismThread.kill();
        sensorState.kill();
        joinAllThreads();
    }

    // ======================== DRIVER (Gamepad 1) ========================

    private void handleDriverControls() {
        // Mecanum Drive
        sensorState.setDriveInput(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x * 0.5
        );

        // LB – Toggle auto-align
        if (gamepad1.left_bumper && !prevLBumper1) {
            sensorState.toggleAutoAlign();
        }
        prevLBumper1 = gamepad1.left_bumper;

        // RB – Reset Pose from AprilTag
        if (gamepad1.right_bumper && !prevRBumper1) {
            if (sensorState.isBasketTagVisible()) {
                sensorState.requestPoseUpdate();
            }
        }
        prevRBumper1 = gamepad1.right_bumper;

        // Intake control
        if (gamepad1.right_trigger > 0.1) {
            mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);
        } else if (gamepad1.left_trigger > 0.1) {
            mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.OUT);
        } else {
            mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
        }
    }

    // ======================== GUNNER (Gamepad 2) ========================

    private void handleGunnerControls() {
        // Bumpers – Shooter spin-up / spin-down
        if (gamepad2.right_bumper) {
            sensorState.setShooterTargetVelocity(DEFAULT_VELOCITY);
        } else if (gamepad2.left_bumper) {
            sensorState.setShooterTargetVelocity(0);
        }

        // A – Shoot GPP sequence (GREEN, PURPLE, PURPLE)
        if (gamepad2.a && !prevA2) {
            ensureShooterSpinning();
            ShootSequence.BallColor[] gppOrder = {
                    ShootSequence.BallColor.GREEN,
                    ShootSequence.BallColor.PURPLE,
                    ShootSequence.BallColor.PURPLE
            };
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.SHOOT_SEQUENCE, gppOrder));
        }
        prevA2 = gamepad2.a;

        // B – Shoot PGP sequence (PURPLE, GREEN, PURPLE)
        if (gamepad2.b && !prevB2) {
            ensureShooterSpinning();
            ShootSequence.BallColor[] pgpOrder = {
                    ShootSequence.BallColor.PURPLE,
                    ShootSequence.BallColor.GREEN,
                    ShootSequence.BallColor.PURPLE
            };
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.SHOOT_SEQUENCE, pgpOrder));
        }
        prevB2 = gamepad2.b;

        // X – Shoot PPG sequence (PURPLE, PURPLE, GREEN)
        if (gamepad2.x && !prevX2) {
            ensureShooterSpinning();
            ShootSequence.BallColor[] ppgOrder = {
                    ShootSequence.BallColor.PURPLE,
                    ShootSequence.BallColor.PURPLE,
                    ShootSequence.BallColor.GREEN
            };
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.SHOOT_SEQUENCE, ppgOrder));
        }
        prevX2 = gamepad2.x;

        // Y – Abort sequence
        if (gamepad2.y && !prevY2) {
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.ABORT_SEQUENCE));
        }
        prevY2 = gamepad2.y;

        // D-Pad Up / Down – Nudge carousel (small adjustment)
        if (gamepad2.dpad_up && !prevDpadUp2) {
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.NUDGE,
                            CarouselController.NUDGE_TICKS));
        }
        prevDpadUp2 = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !prevDpadDown2) {
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.NUDGE,
                            -CarouselController.NUDGE_TICKS));
        }
        prevDpadDown2 = gamepad2.dpad_down;

        // Left Trigger – Shoot GREEN single
        boolean ltPressed = gamepad2.left_trigger > 0.1;
        if (ltPressed && !prevLTrigger2) {
            ensureShooterSpinning();
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.SHOOT_SINGLE,
                            ShootSequence.BallColor.GREEN));
        }
        prevLTrigger2 = ltPressed;

        // Right Trigger – Shoot PURPLE single
        boolean rtPressed = gamepad2.right_trigger > 0.1;
        if (rtPressed && !prevRTrigger2) {
            ensureShooterSpinning();
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.SHOOT_SINGLE,
                            ShootSequence.BallColor.PURPLE));
        }
        prevRTrigger2 = rtPressed;

        // Auto-scale shooter velocity from AprilTag distance
        if (sensorState.getShooterTargetVelocity() > 0 && sensorState.isBasketTagVisible()) {
            sensorState.setVelocityFromDistance(sensorState.getTagRange());
        }
    }

    private void ensureShooterSpinning() {
        if (sensorState.getShooterTargetVelocity() <= 0) {
            sensorState.setShooterTargetVelocity(DEFAULT_VELOCITY);
        }
    }

    // ======================== TELEMETRY ========================

    private void updateTelemetry() {
        telemetry.addData("Runtime", "%.1f", runtime.seconds());
        telemetry.addLine();

        // Ball positions
        telemetry.addData("Carousel", "%s | %s | %s",
                shortName(sensorState.getPositionColor(0)),
                shortName(sensorState.getPositionColor(1)),
                shortName(sensorState.getPositionColor(2)));

        // Shooter status
        telemetry.addData("Shooter", "%.0f / %.0f RPM %s",
                sensorState.getShooterCurrentVelocity(),
                sensorState.getShooterTargetVelocity(),
                sensorState.isShooterReady() ? "✓" : "");

        // Mechanism state
        telemetry.addData("State", mechanismThread.getStateDebug());

        // AprilTag
        if (sensorState.isBasketTagVisible()) {
            telemetry.addData("Tag", "Range: %.1f | Bearing: %.1f",
                    sensorState.getTagRange(),
                    sensorState.getTargetBearing());
        } else {
            telemetry.addData("Tag", "Not Visible");
        }

        // Auto-align status
        telemetry.addData("Auto-Align", sensorState.isAutoAlignEnabled() ? "ON" : "OFF");

        telemetry.update();
    }

    private String shortName(ShootSequence.BallColor c) {
        if (c == ShootSequence.BallColor.GREEN) return "G";
        if (c == ShootSequence.BallColor.PURPLE) return "P";
        return "-";
    }

    private void joinAllThreads() {
        try {
            mechanismThread.join(200);
            driveThread.join(200);
            shooterThread.join(200);
            cameraThread.join(200);
            controlHubI2C.join(200);
            expansionHubI2C.join(200);
        } catch (InterruptedException ignored) {}
    }
}