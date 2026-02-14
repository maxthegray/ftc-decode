package org.firstinspires.ftc.teamcode.Shooter;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.threaded.Old.CameraThread;
import org.firstinspires.ftc.teamcode.threaded.Old.ControlHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.Old.DriveThread;
import org.firstinspires.ftc.teamcode.threaded.Old.ExpansionHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.Old.MechanismThread;
import org.firstinspires.ftc.teamcode.threaded.Old.SensorState;
import org.firstinspires.ftc.teamcode.threaded.Old.ShooterThread;

import java.util.ArrayList;

@Disabled
@TeleOp(name = "Shooter Calibration", group = "Calibration")
public class Calibrate extends LinearOpMode {

    // Use BLUE or RED basket tag — change as needed
    private static final int BASKET_TAG_ID = CameraThread.TAG_BLUE_BASKET;

    private SensorState sensorState;
    private MechanismThread mechanismThread;
    private DriveThread driveThread;
    private ShooterThread shooterThread;
    private CameraThread cameraThread;
    private ControlHubI2CThread controlHubI2C;
    private ExpansionHubI2CThread expansionHubI2C;

    private double targetVelocity = 0.0;

    // Shot log
    private final ArrayList<String> shotLog = new ArrayList<>();
    private int shotCount = 0;

    @Override
    public void runOpMode() {
        sensorState = new SensorState(SensorState.Alliance.BLUE); // Change to RED if needed

        // Initialize threads
        mechanismThread = new MechanismThread(hardwareMap);
        mechanismThread.setSensorState(sensorState);
        driveThread = new DriveThread(sensorState, hardwareMap);
        shooterThread = new ShooterThread(sensorState, hardwareMap);
        cameraThread = new CameraThread(sensorState, hardwareMap, BASKET_TAG_ID);
        controlHubI2C = new ControlHubI2CThread(sensorState, hardwareMap);
        expansionHubI2C = new ExpansionHubI2CThread(sensorState, hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Start all threads
        mechanismThread.start();
        driveThread.start();
        shooterThread.start();
        cameraThread.start();
        controlHubI2C.start();
        expansionHubI2C.start();

        boolean isLauncherOff = false;

        while (opModeIsActive()) {
            // === DRIVING (Gamepad 1) ===
            sensorState.setDriveInput(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x * 0.5
            );

            // LB — Toggle auto-align (uses PID heading lock from DriveThread)
            if (gamepad1.left_bumper) {
                sensorState.toggleAutoAlign();
                sleep(200);
            }

            // RB — Reset pose from AprilTag
            if (gamepad1.right_bumper) {
                if (sensorState.isBasketTagVisible()) sensorState.requestPoseUpdate();
                sleep(200);
            }

            // === SHOOTER POWER (Gamepad 2 D-pad) ===
            if (gamepad2.dpad_up)    { targetVelocity += 1;  sleep(100); isLauncherOff = false; }
            if (gamepad2.dpad_down)  { targetVelocity -= 1;  sleep(100); isLauncherOff = false; }
            if (gamepad2.dpad_right) { targetVelocity += 5;  sleep(100); isLauncherOff = false; }
            if (gamepad2.dpad_left)  { targetVelocity -= 5;  sleep(100); isLauncherOff = false; }

            // Cross — Kill shooter
            if (gamepad2.cross) {
                sensorState.setShooterTargetVelocity(0);
                isLauncherOff = true;
            }

            // Apply target velocity
            if (!isLauncherOff) {
                sensorState.setShooterTargetVelocity(targetVelocity);
            }

            // Square — Kick (and log the shot)
            if (gamepad2.square) {
                logAndKick();
                sleep(200);
            }

            // === INTAKE (Gamepad 2 triggers/bumper) ===
            if (gamepad2.right_trigger > 0.1) {
                mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);
            } else if (gamepad2.left_trigger > 0.1) {
                mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.OUT);
            } else if (gamepad2.right_bumper) {
                mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
            }

            // Keep mechanism thread aware of ball positions
            mechanismThread.setBallPositions(sensorState.getAllPositions());

            // === TELEMETRY ===
            double actual = sensorState.getShooterCurrentVelocity();
            double target = sensorState.getShooterTargetVelocity();
            boolean tagVisible = sensorState.isBasketTagVisible();

            telemetry.addLine("=== SHOOTER CALIBRATION ===");
            telemetry.addData("Target Deg/s", "%.1f", target);
            telemetry.addData("Actual Deg/s", "%.1f", actual);
            telemetry.addData("Error", "%.1f", Math.abs(actual - target));
            telemetry.addData("Shooter Ready", sensorState.isShooterReady() ? "YES" : "NO");
            telemetry.addLine();
            telemetry.addData("Distance", tagVisible ? String.format("%.1f in", sensorState.getTagRange()) : "NO TAG");
            telemetry.addData("Bearing", tagVisible ? String.format("%.1f°", sensorState.getTagBearing()) : "NO TAG");
            telemetry.addData("Auto-align", sensorState.isAutoAlignEnabled() ? "ON" : "OFF");

            telemetry.addLine();
            telemetry.addLine("=== SHOT LOG ===");
            int start = Math.max(0, shotLog.size() - 10);
            for (int i = start; i < shotLog.size(); i++) {
                telemetry.addLine(shotLog.get(i));
            }

            telemetry.update();
        }

        // Cleanup
        mechanismThread.kill();
        sensorState.kill();
        joinAllThreads();
    }

    /**
     * Log actual velocity + distance at time of shot, then kick.
     */
    private void logAndKick() {
        shotCount++;
        double actualVelocity = sensorState.getShooterCurrentVelocity();
        double targetVel = sensorState.getShooterTargetVelocity();
        boolean tagVisible = sensorState.isBasketTagVisible();

        String distStr = tagVisible ? String.format("%.1f in", sensorState.getTagRange()) : "NO TAG";
        String bearingStr = tagVisible ? String.format("%.1f°", sensorState.getTagBearing()) : "N/A";

        String entry = String.format("#%d | target: %.1f | actual: %.1f deg/s | dist: %s | bearing: %s",
                shotCount, targetVel, actualVelocity, distStr, bearingStr);
        shotLog.add(entry);

        // Send kick command to MechanismThread
        mechanismThread.enqueueCommand(
                new MechanismThread.Command(MechanismThread.Command.Type.KICK));
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