package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.threaded.Old.CameraThread;
import org.firstinspires.ftc.teamcode.threaded.Old.ControlHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.Old.ExpansionHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.Old.MechanismThread;
import org.firstinspires.ftc.teamcode.threaded.Old.SensorState;
import org.firstinspires.ftc.teamcode.threaded.Old.ShooterThread;
import org.firstinspires.ftc.teamcode.threaded.Old.ShootSequence;

/**
 * INTAKE + SHOOT AUTO (Blue Alliance)
 *
 * Flow:
 *   1. Start at startPose (camera reads shoot-order AprilTag here)
 *   2. Drive to pose2 (shooting position)
 *   3. Drive to pose3 (middle balls area)
 *   4. Run 3-ball intake sequence using BALL_1 / BALL_2 / BALL_3
 *   5. Drive back to pose2
 *   6. Spin up shooter, shoot all 3 balls in AprilTag-detected order
 *
 * Intake sequence per ball:
 *   DRIVE_TO_BALL (intake ON) → detected? → WAIT_AUTO_INDEX → PAUSE_BETWEEN →
 *   DRIVE_TO_BALL (next) → ... → start shoot phase
 *                          └─ arrived, no ball → LINGER_FOR_BALL (intake still ON)
 */
@Autonomous(name = "Intake Tuning Auto", group = "Tuning")
@Configurable
public class IntakeTuningAuto extends OpMode {

    // ======================== ALLIANCE CONFIG ========================

    private static final int BASKET_TAG_ID = CameraThread.TAG_BLUE_BASKET;

    // Default shoot order if AprilTag is never detected (GPP)
    private static final ShootSequence.BallColor[] DEFAULT_SHOOT_ORDER = {
            ShootSequence.BallColor.GREEN,
            ShootSequence.BallColor.PURPLE,
            ShootSequence.BallColor.PURPLE
    };

    // ======================== ROUTE POSES ========================

    private final Pose startPose = new Pose(48, 9, Math.toRadians(90));    // starting pose
    private final Pose pose2 = new Pose(60, 16, Math.toRadians(115));      // shoot position
    private final Pose pose3 = new Pose(48, 57, Math.toRadians(180));      // middle balls

    // ======================== TUNABLE BALL PICKUP POSES ========================

    public static Pose BALL_1 = new Pose(43, 57, Math.toRadians(180));
    public static Pose BALL_2 = new Pose(38, 57, Math.toRadians(180));
    public static Pose BALL_3 = new Pose(33, 57, Math.toRadians(180));

    // ======================== TUNABLE TIMING ========================

    /** Max time to wait for a ball AFTER arriving at the pose (ms). */
    public static long BALL_LINGER_TIMEOUT_MS = 1500;

    /** Extra pause after auto-index finishes before driving to next ball (ms). */
    public static long PAUSE_AFTER_INDEX_MS = 300;

    /** Time to let intake spin up before the robot starts driving to the ball (ms). */
    public static long INTAKE_SPIN_UP_MS = 400;

    /** Shooter velocity to use if tag-based distance scaling isn't available. */
    public static double DEFAULT_SHOOTER_VELOCITY = 150;

    /** Max time to wait for the shooter to spin up before shooting anyway (ms). */
    public static long SHOOTER_SPINUP_TIMEOUT_MS = 3000;

    /** Max time to wait for the shoot sequence to complete (ms). */
    public static long SHOOT_SEQUENCE_TIMEOUT_MS = 15000;

    // ======================== INTERNALS ========================

    private enum State {
        // --- Phase 1: Drive to shooting position then to ball area ---
        DRIVE_TO_POSE2,
        DRIVE_TO_POSE3,

        // --- Phase 2: Intake 3 balls ---
        SPIN_UP_INTAKE,
        DRIVE_TO_BALL,
        LINGER_FOR_BALL,
        WAIT_AUTO_INDEX,
        PAUSE_BETWEEN,

        // --- Phase 3: Drive back and shoot ---
        DRIVE_TO_SHOOT,         // Driving back to pose2
        SHOOTER_SPINUP,         // Waiting for shooter to reach target velocity
        SHOOTING,               // SHOOT_SEQUENCE command issued, waiting for completion
        DONE
    }

    private State state = State.DRIVE_TO_POSE2;
    private int currentBall = 0; // 0, 1, 2
    private final Pose[] ballPoses = new Pose[3];

    // Shoot order captured from AprilTag (or default)
    private ShootSequence.BallColor[] shootOrder = null;

    private Follower follower;
    private MechanismThread mechanismThread;
    private SensorState sensorState;
    private CameraThread cameraThread;
    private ShooterThread shooterThread;
    private ControlHubI2CThread controlHubI2C;
    private ExpansionHubI2CThread expansionHubI2C;

    private Timer stateTimer;
    private Timer opmodeTimer;
    private TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        stateTimer = new Timer();
        opmodeTimer = new Timer();

        // Build hardware
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        sensorState = new SensorState();
        mechanismThread = new MechanismThread(hardwareMap);
        mechanismThread.setSensorState(sensorState);
        controlHubI2C = new ControlHubI2CThread(sensorState, hardwareMap);
        expansionHubI2C = new ExpansionHubI2CThread(sensorState, hardwareMap);
        shooterThread = new ShooterThread(sensorState, hardwareMap);

        // Start camera immediately so it can read the shoot-order tag from startPose
        cameraThread = new CameraThread(sensorState, hardwareMap, BASKET_TAG_ID);
        cameraThread.start();

        // Enable auto-index for the intake phase
        mechanismThread.enqueueCommand(
                new MechanismThread.Command(MechanismThread.Command.Type.SET_AUTO_INDEX, true));

        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        // Show whether shoot-order tag has been detected during init
        boolean hasOrder = sensorState.hasDetectedShootOrder();
        panelsTelemetry.debug("Shoot Order Tag", hasOrder ? "DETECTED" : "waiting...");

        if (hasOrder) {
            ShootSequence.BallColor[] order = sensorState.getDetectedShootOrder();
            panelsTelemetry.debug("Shoot Order",
                    shortColor(order[0]) + " " + shortColor(order[1]) + " " + shortColor(order[2]));
        }

        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        // Snapshot configurable poses
        ballPoses[0] = BALL_1;
        ballPoses[1] = BALL_2;
        ballPoses[2] = BALL_3;

        // Capture shoot order now (tag should have been read during init)
        if (sensorState.hasDetectedShootOrder()) {
            shootOrder = sensorState.getDetectedShootOrder();
        } else {
            shootOrder = DEFAULT_SHOOT_ORDER.clone();
        }

        opmodeTimer.resetTimer();
        stateTimer.resetTimer();

        // Start remaining threads
        mechanismThread.start();
        controlHubI2C.start();
        expansionHubI2C.start();
        shooterThread.start();

        // Drive to pose2 first
        driveToPose(pose2);
        state = State.DRIVE_TO_POSE2;
    }

    @Override
    public void loop() {
        // Always update follower and feed sensor data to mechanism thread
        follower.update();
        mechanismThread.setBallPositions(sensorState.getAllPositions());

        switch (state) {

            // ====================== PHASE 1: NAVIGATE ======================

            case DRIVE_TO_POSE2:
                if (!follower.isBusy()) {
                    driveToPose(pose3);
                    state = State.DRIVE_TO_POSE3;
                }
                break;

            case DRIVE_TO_POSE3:
                if (!follower.isBusy()) {
                    currentBall = 0;
                    mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);
                    stateTimer.resetTimer();
                    state = State.SPIN_UP_INTAKE;
                }
                break;

            // ====================== PHASE 2: INTAKE 3 BALLS ======================

            case SPIN_UP_INTAKE:
                if (stateTimer.getElapsedTimeSeconds() * 1000 >= INTAKE_SPIN_UP_MS) {
                    driveToBall(currentBall);
                    state = State.DRIVE_TO_BALL;
                }
                break;

            case DRIVE_TO_BALL: {
                if (checkBallDetected()) {
                    stateTimer.resetTimer();
                    state = State.WAIT_AUTO_INDEX;
                    break;
                }
                if (!follower.isBusy()) {
                    stateTimer.resetTimer();
                    state = State.LINGER_FOR_BALL;
                }
                break;
            }

            case LINGER_FOR_BALL: {
                if (checkBallDetected()) {
                    stateTimer.resetTimer();
                    state = State.WAIT_AUTO_INDEX;
                    break;
                }
                if (stateTimer.getElapsedTimeSeconds() * 1000 >= BALL_LINGER_TIMEOUT_MS) {
                    mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
                    advanceToNextBall();
                }
                break;
            }

            case WAIT_AUTO_INDEX: {
                ShootSequence.BallColor intakeColor = sensorState.getPositionColor(SensorState.POS_INTAKE);
                boolean intakeCleared = (intakeColor == ShootSequence.BallColor.EMPTY);
                boolean isFull = (countBalls() >= 3);
                String mechState = mechanismThread.getStateDebug();
                boolean mechIdle = mechState.contains("IDLE");

                if (intakeCleared || isFull || (mechIdle && stateTimer.getElapsedTimeSeconds() > 0.5)) {
                    mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
                    stateTimer.resetTimer();
                    state = State.PAUSE_BETWEEN;
                }

                // Safety timeout
                if (stateTimer.getElapsedTimeSeconds() * 1000 >= 3000) {
                    mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
                    stateTimer.resetTimer();
                    state = State.PAUSE_BETWEEN;
                }
                break;
            }

            case PAUSE_BETWEEN:
                if (stateTimer.getElapsedTimeSeconds() * 1000 >= PAUSE_AFTER_INDEX_MS) {
                    advanceToNextBall();
                }
                break;

            // ====================== PHASE 3: DRIVE BACK AND SHOOT ======================

            case DRIVE_TO_SHOOT:
                // Start spinning up the shooter while driving back
                if (sensorState.getShooterTargetVelocity() <= 0) {
                    sensorState.setShooterTargetVelocity(DEFAULT_SHOOTER_VELOCITY);
                }

                // Scale velocity from AprilTag distance if visible
                if (sensorState.isBasketTagVisible()) {
                    sensorState.setVelocityFromDistance(sensorState.getTagRange());
                }

                if (!follower.isBusy()) {
                    stateTimer.resetTimer();
                    state = State.SHOOTER_SPINUP;
                }
                break;

            case SHOOTER_SPINUP:
                // Keep updating velocity from tag if visible
                if (sensorState.isBasketTagVisible()) {
                    sensorState.setVelocityFromDistance(sensorState.getTagRange());
                }

                boolean shooterReady = sensorState.isShooterReady();
                boolean spinupTimedOut = stateTimer.getElapsedTimeSeconds() * 1000 >= SHOOTER_SPINUP_TIMEOUT_MS;

                if (shooterReady || spinupTimedOut) {
                    // Disable auto-index, switch to shoot mode
                    mechanismThread.enqueueCommand(
                            new MechanismThread.Command(
                                    MechanismThread.Command.Type.SET_AUTO_INDEX, false));

                    // Issue the shoot sequence command
                    mechanismThread.enqueueCommand(
                            new MechanismThread.Command(
                                    MechanismThread.Command.Type.SHOOT_SEQUENCE, shootOrder));

                    stateTimer.resetTimer();
                    state = State.SHOOTING;
                }
                break;

            case SHOOTING:
                // Keep updating velocity from tag while shooting
                if (sensorState.isBasketTagVisible()) {
                    sensorState.setVelocityFromDistance(sensorState.getTagRange());
                }

                // Check if mechanism is done shooting (back to IDLE with no shots left)
                String mechDebug = mechanismThread.getStateDebug();
                boolean mechIdle = mechDebug.contains("IDLE");
                boolean timedOut = stateTimer.getElapsedTimeSeconds() * 1000 >= SHOOT_SEQUENCE_TIMEOUT_MS;

                // Give mechanism a moment to start before checking IDLE
                boolean startupGrace = stateTimer.getElapsedTimeSeconds() < 0.5;

                if ((!startupGrace && mechIdle) || timedOut) {
                    sensorState.setShooterTargetVelocity(0);
                    state = State.DONE;
                }
                break;

            case DONE:
                break;
        }

        // ======================== TELEMETRY ========================
        panelsTelemetry.debug("State", state.name());

        if (state.ordinal() <= State.PAUSE_BETWEEN.ordinal()) {
            panelsTelemetry.debug("Ball #", (currentBall + 1) + " / 3");
        }

        panelsTelemetry.debug("Balls Held",
                shortColor(sensorState.getPositionColor(0)) + " | " +
                        shortColor(sensorState.getPositionColor(1)) + " | " +
                        shortColor(sensorState.getPositionColor(2)));
        panelsTelemetry.debug("Ball Count", countBalls());
        panelsTelemetry.debug("Shoot Order",
                shortColor(shootOrder[0]) + " " +
                        shortColor(shootOrder[1]) + " " +
                        shortColor(shootOrder[2]));
        panelsTelemetry.debug("Shooter", String.format("%.0f / %.0f",
                sensorState.getShooterCurrentVelocity(),
                sensorState.getShooterTargetVelocity()));
        panelsTelemetry.debug("Mech State", mechanismThread.getStateDebug());
        panelsTelemetry.debug("Tag Visible", sensorState.isBasketTagVisible() ? "YES" : "no");
        panelsTelemetry.debug("State Timer", String.format("%.1fs", stateTimer.getElapsedTimeSeconds()));
        panelsTelemetry.debug("Total Time", String.format("%.1fs", opmodeTimer.getElapsedTimeSeconds()));
        panelsTelemetry.debug("Pose", String.format("(%.1f, %.1f) %.0f°",
                follower.getPose().getX(), follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading())));
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
        sensorState.setShooterTargetVelocity(0);
        mechanismThread.kill();
        sensorState.kill();
        try {
            mechanismThread.join(300);
            controlHubI2C.join(300);
            expansionHubI2C.join(300);
            shooterThread.join(300);
            cameraThread.join(300);
        } catch (InterruptedException ignored) {}
    }

    // ======================== HELPERS ========================

    /** Drive to a pose (no intake). */
    private void driveToPose(Pose target) {
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), target))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), target.getHeading())
                .build();
        follower.followPath(path, true);
    }

    /** Check if a ball is currently detected at the intake position. */
    private boolean checkBallDetected() {
        ShootSequence.BallColor intakeColor = sensorState.getPositionColor(SensorState.POS_INTAKE);
        return intakeColor == ShootSequence.BallColor.GREEN
                || intakeColor == ShootSequence.BallColor.PURPLE;
    }

    /** Drive to the ball at the given index (intake should already be running). */
    private void driveToBall(int index) {
        Pose target = ballPoses[index];
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), target))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), target.getHeading())
                .build();
        follower.followPath(path, true);
    }

    /** Move to the next ball, or transition to shoot phase if all 3 are done. */
    private void advanceToNextBall() {
        currentBall++;
        if (currentBall >= 3 || countBalls() >= 3) {
            // All balls collected (or attempted) — drive back to shoot
            mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
            driveToPose(pose2);

            // Start spinning up the shooter early while driving
            sensorState.setShooterTargetVelocity(DEFAULT_SHOOTER_VELOCITY);

            state = State.DRIVE_TO_SHOOT;
            return;
        }

        // Spin up intake first, then drive
        mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);
        stateTimer.resetTimer();
        state = State.SPIN_UP_INTAKE;
    }

    private int countBalls() {
        int count = 0;
        for (ShootSequence.BallColor c : sensorState.getAllPositions()) {
            if (c == ShootSequence.BallColor.GREEN || c == ShootSequence.BallColor.PURPLE) count++;
        }
        return count;
    }

    private String shortColor(ShootSequence.BallColor c) {
        if (c == ShootSequence.BallColor.GREEN) return "G";
        if (c == ShootSequence.BallColor.PURPLE) return "P";
        return "-";
    }
}