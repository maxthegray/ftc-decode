package org.firstinspires.ftc.teamcode.threaded.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.threaded.CameraThread;
import org.firstinspires.ftc.teamcode.threaded.CarouselController;
import org.firstinspires.ftc.teamcode.threaded.ControlHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.DriveThread;
import org.firstinspires.ftc.teamcode.threaded.ExpansionHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.MechanismThread;
import org.firstinspires.ftc.teamcode.threaded.SensorState;
import org.firstinspires.ftc.teamcode.threaded.ShootSequence;
import org.firstinspires.ftc.teamcode.threaded.ShooterThread;

@TeleOp(name = "Blue", group = "TeleOp")
public class TeleOpBlue extends LinearOpMode {

    private static final int BASKET_TAG_ID = CameraThread.TAG_BLUE_BASKET;
    private static final double DEFAULT_VELOCITY = 150;

    public enum RobotMode {
        INTAKING,
        SHOOTING
    }
    private RobotMode currentMode = RobotMode.INTAKING;

    private MechanismThread mechanismThread;
    private DriveThread driveThread;
    private ShooterThread shooterThread;
    private CameraThread cameraThread;
    private ControlHubI2CThread controlHubI2C;
    private ExpansionHubI2CThread expansionHubI2C;
    private SensorState sensorState;

    private ElapsedTime runtime;

    private boolean prevLBumper1 = false;
    private boolean prevRBumper1 = false;

    private boolean prevX2 = false;
    private boolean prevDpadLeft2 = false;
    private boolean prevDpadRight2 = false;
    private boolean prevLTrigger2 = false;
    private boolean prevRTrigger2 = false;
    private boolean prevB2 = false;

    @Override
    public void runOpMode() {
        runtime = new ElapsedTime();
        sensorState = new SensorState(SensorState.Alliance.BLUE);

        mechanismThread = new MechanismThread(hardwareMap);
        mechanismThread.setSensorState(sensorState);
        mechanismThread.setSkipKickback(true);
        driveThread = new DriveThread(sensorState, hardwareMap);
        shooterThread = new ShooterThread(sensorState, hardwareMap);
        cameraThread = new CameraThread(sensorState, hardwareMap, BASKET_TAG_ID);
        controlHubI2C = new ControlHubI2CThread(sensorState, hardwareMap);
        expansionHubI2C = new ExpansionHubI2CThread(sensorState, hardwareMap);

        telemetry.addData("Status", "Initialized");
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
            handleAutoModeSwitching();

            updateTelemetry();
        }

        mechanismThread.kill();
        sensorState.kill();
        joinAllThreads();
    }

    private void handleDriverControls() {
        sensorState.setDriveInput(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x * 0.75
        );

        if (gamepad1.left_bumper && !prevLBumper1) sensorState.toggleAutoAlign();
        prevLBumper1 = gamepad1.left_bumper;

        if (gamepad1.right_bumper && !prevRBumper1) {
            if (sensorState.isBasketTagVisible()) sensorState.requestPoseUpdate();
        }
        prevRBumper1 = gamepad1.right_bumper;

        boolean carouselSettled = mechanismThread.isCarouselSettled();

        if (gamepad1.right_trigger > 0.1 && carouselSettled) {
            mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);
            sensorState.setShooterTargetVelocity(0);
            if (currentMode != RobotMode.INTAKING) switchMode(RobotMode.INTAKING);
        } else if (gamepad1.left_trigger > 0.1) {
            mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.OUT);
            sensorState.setShooterTargetVelocity(0);
        } else {
            mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
        }
    }

    private void handleGunnerControls() {
        boolean intakeActive = gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1;

        if (gamepad2.right_bumper && !intakeActive) {
            sensorState.setShooterTargetVelocity(DEFAULT_VELOCITY);
        } else if (gamepad2.left_bumper || intakeActive) {
            sensorState.setShooterTargetVelocity(0);
        }

        if (gamepad2.x && !prevX2) {
            switchMode(RobotMode.SHOOTING);
            ensureShooterSpinning();
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.KICK));
        }
        prevX2 = gamepad2.x;

        boolean ltPressed = gamepad2.left_trigger > 0.1;
        if (ltPressed && !prevLTrigger2) {
            switchMode(RobotMode.SHOOTING);
            ensureShooterSpinning();
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.SHOOT_SINGLE,
                            ShootSequence.BallColor.GREEN));
        }
        prevLTrigger2 = ltPressed;

        boolean rtPressed = gamepad2.right_trigger > 0.1;
        if (rtPressed && !prevRTrigger2) {
            switchMode(RobotMode.SHOOTING);
            ensureShooterSpinning();
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.SHOOT_SINGLE,
                            ShootSequence.BallColor.PURPLE));
        }
        prevRTrigger2 = rtPressed;

        if (gamepad2.dpad_left && !prevDpadLeft2) {
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.ROTATE_LEFT));
        }
        prevDpadLeft2 = gamepad2.dpad_left;

        if (gamepad2.dpad_right && !prevDpadRight2) {
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.ROTATE_RIGHT));
        }
        prevDpadRight2 = gamepad2.dpad_right;

        if (gamepad2.dpad_up) {
            mechanismThread.setNudgeRequest(CarouselController.NUDGE_TICKS);
        } else if (gamepad2.dpad_down) {
            mechanismThread.setNudgeRequest(-CarouselController.NUDGE_TICKS);
        } else {
            mechanismThread.setNudgeRequest(0);
        }

        if (gamepad2.circle && !prevB2) {
            mechanismThread.enqueueCommand(
                    new MechanismThread.Command(MechanismThread.Command.Type.SHOW_LIGHTS));
        }
        prevB2 = gamepad2.circle;

        if (!intakeActive && sensorState.getShooterTargetVelocity() > 0 && sensorState.isBasketTagVisible()) {
            sensorState.setVelocityFromDistance(sensorState.getTagRange());
        }
    }

    private void ensureShooterSpinning() {
        boolean intakeActive = gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1;
        if (!intakeActive && sensorState.getShooterTargetVelocity() <= 0) {
            sensorState.setShooterTargetVelocity(DEFAULT_VELOCITY);
        }
    }

    private void handleAutoModeSwitching() {
        int ballCount = 0;
        for (ShootSequence.BallColor c : sensorState.getAllPositions()) {
            if (c != ShootSequence.BallColor.EMPTY) ballCount++;
        }

        if (ballCount >= 3 && currentMode == RobotMode.INTAKING) {
            switchMode(RobotMode.SHOOTING);
        }
    }

    private void switchMode(RobotMode mode) {
        currentMode = mode;
        boolean enableAuto = (mode == RobotMode.INTAKING);
        mechanismThread.enqueueCommand(
                new MechanismThread.Command(MechanismThread.Command.Type.SET_AUTO_INDEX, enableAuto));
    }

    private void updateTelemetry() {
        telemetry.addData("Mode", currentMode);
        telemetry.addData("Balls", "%s|%s|%s",
                shortName(sensorState.getPositionColor(0)),
                shortName(sensorState.getPositionColor(1)),
                shortName(sensorState.getPositionColor(2)));

        double current = sensorState.getShooterCurrentVelocity();
        double target = sensorState.getShooterTargetVelocity();
        double error = Math.abs(current - target);
        boolean ready = sensorState.isShooterReady();
        telemetry.addData("Shooter", "%.0f / %.0f RPM (err %.0f) %s",
                current, target, error, ready ? "READY" : "NOT READY");

        boolean settled = mechanismThread.isCarouselSettled();
        telemetry.addData("Carousel", "curr: %d / target: %d | %s",
                mechanismThread.getCarouselCurrentTicks(),
                mechanismThread.getCarouselTargetTicks(),
                settled ? "SETTLED" : "MOVING");

        telemetry.addData("CmdReady", "idle=%b settled=%b",
                mechanismThread.isIdle(), settled);

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