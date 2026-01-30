package org.firstinspires.ftc.teamcode.threaded.Old.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.threaded.Old.BotState;
import org.firstinspires.ftc.teamcode.threaded.Old.CameraThread;
import org.firstinspires.ftc.teamcode.threaded.Old.CarouselThread;
import org.firstinspires.ftc.teamcode.threaded.Old.DriveThread;
import org.firstinspires.ftc.teamcode.threaded.Old.ShooterThread;

@Autonomous(name = "Auto - Shoot 3 (BLUE)", group = "Auto")
public class AutoBlue extends LinearOpMode {

    private static final int BASKET_TAG_ID = CameraThread.TAG_BLUE_BASKET;

    private static final double DRIVE_POWER = 0.5;         // forward speed
    private static final double DRIVE_TIME_SEC = 1.0;      // drive duration

    private static final double DEFAULT_VELOCITY = 0;  // if tag not visible
    private static final long TAG_TIMEOUT_MS = 3000;       // wait for tag
    private static final long SPIN_UP_MS = 1500;           // time to spin up shooter
    private static final long POST_KICK_MS = 500;          // wait after kick
    private static final long CAROUSEL_SETTLE_MS = 800;    // wait for carousel to rotate

    private BotState state;
    private DriveThread driveThread;
    private CarouselThread carouselThread;
    private ShooterThread shooterThread;
    private CameraThread cameraThread;

    @Override
    public void runOpMode() {
        // Initialize
        state = new BotState();
        state.setAutoIndexEnabled(false);

        driveThread = new DriveThread(state, hardwareMap);
        carouselThread = new CarouselThread(state, hardwareMap);
        shooterThread = new ShooterThread(state, hardwareMap);
        cameraThread = new CameraThread(state, hardwareMap, BASKET_TAG_ID);

        telemetry.addData("Alliance", "BLUE (Tag %d)", BASKET_TAG_ID);
        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        // Start threads
        driveThread.start();
        carouselThread.start();
        shooterThread.start();
        cameraThread.start();

        // === STEP 1: Drive forward for 5 seconds ===
        ElapsedTime timer = new ElapsedTime();



        // Stop driving


        // === STEP 2: Wait for tag to get distance ===
        telemetry.addData("Step", "Looking for tag...");
        telemetry.update();

        timer.reset();
        while (opModeIsActive() && !state.isBasketTagVisible() && timer.milliseconds() < TAG_TIMEOUT_MS) {
            telemetry.addData("Step", "Looking for tag...");
            telemetry.addData("Time", "%.1f / %.1f sec", timer.seconds(), TAG_TIMEOUT_MS / 1000.0);
            telemetry.update();
            safeSleep(50);
        }

        // Set shooter velocity based on distance
        if (state.isBasketTagVisible()) {
            double range = state.getTagRange();
            state.setAdjustedVelocity(range);
            sleep(500);
            state.setShooterTargetVelocity(state.getShooterTargetVelocity() - 2);
            telemetry.addData("Range", "%.1f in", range);
            telemetry.addData("Velocity", "%.0f deg/s", state.getShooterTargetVelocity());
            safeSleep(1000);

        } else {
            state.setShooterTargetVelocity(DEFAULT_VELOCITY);

            telemetry.addData("Range", "TAG NOT FOUND - using default");
            telemetry.addData("Velocity", "%.0f deg/s", DEFAULT_VELOCITY);
        }
        telemetry.update();

        // === STEP 3: Spin up shooter ===
        telemetry.addData("Step", "Spinning up...");
        telemetry.update();
        safeSleep(SPIN_UP_MS);

        // === STEP 4: Shoot 3 balls ===
        for (int i = 1; i <= 3; i++) {


            telemetry.addData("Step", "Shooting ball %d/3", i);
            telemetry.addData("Velocity", "%.0f deg/s", state.getShooterTargetVelocity());
            telemetry.update();

            // Wait for shooter ready
            while (opModeIsActive() && !state.isShooterReady()) {
                safeSleep(20);
            }

            // Kick
            state.requestKick();

            // Wait for kick to complete
            safeSleep(POST_KICK_MS);
            while (opModeIsActive() && state.isKickerUp()) {
                safeSleep(20);
            }

            // Rotate carousel for next ball (except after last shot)
            if (i < 3) {
                state.setCarouselCommand(BotState.CarouselCommand.ROTATE_RIGHT);
                safeSleep(CAROUSEL_SETTLE_MS);
                while (opModeIsActive() && !state.isCarouselSettled()) {
                    safeSleep(20);
                }
            }
        }

        timer.reset();

        while (timer.seconds() < DRIVE_TIME_SEC) {
            state.setDriveInput(DRIVE_POWER, 0, 0);

            telemetry.addData("Step", "Driving forward");
            telemetry.addData("Time", "%.1f / %.1f sec", timer.seconds(), DRIVE_TIME_SEC);
            telemetry.update();
        }
        state.setDriveInput(0, 0, 0);
        safeSleep(200);

        // Stop shooter
        state.setShooterTargetVelocity(0);

        telemetry.addData("Step", "Done!");
        telemetry.update();

        // Cleanup
        state.endThreads();
        try {
            driveThread.join(500);
            carouselThread.join(500);
            shooterThread.join(500);
            cameraThread.join(500);
        } catch (InterruptedException e) {
            // Ignore
        }
    }

    private void safeSleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}