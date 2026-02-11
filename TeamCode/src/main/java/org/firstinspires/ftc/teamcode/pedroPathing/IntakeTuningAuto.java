package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.threaded.Old.CameraThread;
import org.firstinspires.ftc.teamcode.threaded.Old.ControlHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.Old.ExpansionHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.Old.MechanismThread;
import org.firstinspires.ftc.teamcode.threaded.Old.SensorState;
import org.firstinspires.ftc.teamcode.threaded.Old.ShooterThread;
import org.firstinspires.ftc.teamcode.threaded.Old.ShootSequence;

/**
 * FULL AUTO (Blue Alliance)
 *
 * Flow:
 *   Shoot 0: Start preloaded → drive to pose2 → auto-align → shoot 3
 *   Intake 0: Drive to pose3 (Y=57) → pick up 3 balls (row 1)
 *   Shoot 1: Drive to pose4 → auto-align → shoot 3
 *   Intake 1: Drive to pose5 (Y=83) → pick up 3 balls (row 2, same X as row 1)
 *   Shoot 2: Drive to pose4 → auto-align → shoot 3
 *   Done.
 *
 * During all shoot phases, the robot uses AprilTag auto-align PID (same as teleop)
 * to hold heading on the basket target while the shooter fires.
 */
@Autonomous(name = "Intake Tuning Auto", group = "Tuning")
@Configurable
public class IntakeTuningAuto extends OpMode {

    // ======================== ALLIANCE CONFIG ========================

    private static final int BASKET_TAG_ID = CameraThread.TAG_BLUE_BASKET;

    private static final ShootSequence.BallColor[] DEFAULT_SHOOT_ORDER = {
            ShootSequence.BallColor.GREEN,
            ShootSequence.BallColor.PURPLE,
            ShootSequence.BallColor.PURPLE
    };

    // ======================== ROUTE POSES ========================

    private final Pose startPose = new Pose(48, 9, Math.toRadians(90));
    private final Pose pose2 = new Pose(58, 83, Math.toRadians(140));      // first shoot REPLACING WITH second
    private final Pose pose3 = new Pose(48, 57, Math.toRadians(180));      // ball area row 1
    private final Pose pose4 = new Pose(58, 83, Math.toRadians(140));      // second/third shoot
    private final Pose pose5 = new Pose(50, 83, Math.toRadians(180));      // ball area row 2

    // ======================== TUNABLE BALL PICKUP POSES ========================

    // Row 1 (Y = 57)
    public static Pose BALL_1 = new Pose(43, 57, Math.toRadians(180));
    public static Pose BALL_2 = new Pose(38, 57, Math.toRadians(180));
    public static Pose BALL_3 = new Pose(23, 57, Math.toRadians(180));

    // Row 2 (Y = 83, same X values)
    public static Pose BALL_4 = new Pose(43, 83, Math.toRadians(180));
    public static Pose BALL_5 = new Pose(38, 83, Math.toRadians(180));
    public static Pose BALL_6 = new Pose(30, 83, Math.toRadians(180));

    // ======================== TUNABLE CURVED PATH CONTROL POINTS ========================

    /** Control point for the BezierCurve from start → pose2 */
    public static Pose START_TO_SHOOT0_CONTROL = new Pose(75, 39);

    /** Control point for the BezierCurve from row 1 → pose4 */
    public static Pose ROW1_TO_SHOOT_CONTROL = new Pose(45, 44);

    /** Control point for the BezierCurve from first shoot → ball area row 1 */
    public static Pose SHOOT1_TO_ROW1_CONTROL = new Pose(61, 50);

    // ======================== TUNABLE TIMING ========================

    public static long BALL_LINGER_TIMEOUT_MS = 1500;
    public static long PAUSE_AFTER_INDEX_MS = 300;
    public static long INTAKE_SPIN_UP_MS = 1000;
    public static double DEFAULT_SHOOTER_VELOCITY = 130;
    public static long BALL_AREA_SETTLE_DELAY_MS = 500;
    public static long SHOOTER_SPINUP_TIMEOUT_MS = 3000;
    public static long SHOOT_SEQUENCE_TIMEOUT_MS = 15000;

    /** Max time to wait for auto-align before shooting anyway (ms). */
    public static long ALIGN_TIMEOUT_MS = 2000;

    // ======================== STATE MACHINE ========================

    private enum State {
        // --- Shoot phase (reused for all 3 shoots) ---
        DRIVE_TO_SHOOT,
        ALIGN_AND_SPINUP,       // Teleop mode: PID aligns heading + shooter spins up
        SHOOTING,               // Teleop mode: PID maintains heading while shooting

        // --- Intake phase (reused for both intake cycles) ---
        DRIVE_TO_BALL_AREA,
        SETTLE_AT_BALL_AREA,
        SPIN_UP_INTAKE,
        DRIVE_TO_BALL,
        LINGER_FOR_BALL,
        WAIT_AUTO_INDEX,
        PAUSE_BETWEEN,

        DONE
    }

    private State state = State.DRIVE_TO_SHOOT;

    // Cycle tracking
    private int shootCycle = 0;     // 0 = from pose2, 1 = from pose4, 2 = from pose4
    private int intakeCycle = 0;    // 0 = row 1 (Y=57), 1 = row 2 (Y=83)

    // Shoot poses per cycle
    private final Pose[] shootPoses = new Pose[3];
    private final Pose[] ballAreaPoses = new Pose[2];
    private final Pose[][] ballPoseRows = new Pose[2][3];

    // Current intake ball index within a row (0, 1, 2)
    private int currentBall = 0;

    // Whether we're in teleop drive mode (auto-align active)
    private boolean inTeleOpMode = false;

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

    // ======================== AUTO-ALIGN PID ========================
    // Mirrors DriveThread PID logic using SensorState constants

    private double integralSum = 0;
    private double lastError = 0;
    private boolean hasLastError = false;
    private final ElapsedTime pidTimer = new ElapsedTime();

    private static final double INTEGRAL_LIMIT = 0.3;
    private static final double OUTPUT_MIN = -1.0;
    private static final double OUTPUT_MAX = 1.0;

    // ======================== LIFECYCLE ========================

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        stateTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        sensorState = new SensorState(SensorState.Alliance.BLUE);
        mechanismThread = new MechanismThread(hardwareMap);
        mechanismThread.setSensorState(sensorState);
        controlHubI2C = new ControlHubI2CThread(sensorState, hardwareMap);
        expansionHubI2C = new ExpansionHubI2CThread(sensorState, hardwareMap);
        shooterThread = new ShooterThread(sensorState, hardwareMap);

        // Start camera early so it can read the shoot-order tag from startPose
        cameraThread = new CameraThread(sensorState, hardwareMap, BASKET_TAG_ID);
        cameraThread.start();

        // Auto-index OFF initially — we're shooting first, not intaking
        mechanismThread.enqueueCommand(
                new MechanismThread.Command(MechanismThread.Command.Type.SET_AUTO_INDEX, false));

        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
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
        shootPoses[0] = pose2;
        shootPoses[1] = pose4;
        shootPoses[2] = pose4;

        ballAreaPoses[0] = pose3;
        ballAreaPoses[1] = pose5;

        ballPoseRows[0] = new Pose[] { BALL_1, BALL_2, BALL_3 };
        ballPoseRows[1] = new Pose[] { BALL_4, BALL_5, BALL_6 };

        // Capture shoot order
        if (sensorState.hasDetectedShootOrder()) {
            shootOrder = sensorState.getDetectedShootOrder();
        } else {
            shootOrder = DEFAULT_SHOOT_ORDER.clone();
        }

        opmodeTimer.resetTimer();
        stateTimer.resetTimer();

        // Start threads
        mechanismThread.start();
        controlHubI2C.start();
        expansionHubI2C.start();
        shooterThread.start();

        // Start spinning up shooter immediately while driving to first shoot
        sensorState.setShooterTargetVelocity(DEFAULT_SHOOTER_VELOCITY);

        shootCycle = 0;
        intakeCycle = 0;
        inTeleOpMode = false;
        driveCurvedToFirstShoot(shootPoses[0]);
        state = State.DRIVE_TO_SHOOT;
    }

    @Override
    public void loop() {
        mechanismThread.setBallPositions(sensorState.getAllPositions());

        switch (state) {

            // ==================== SHOOT PHASE ====================

            case DRIVE_TO_SHOOT:
                updateShooterFromTag();
                if (!follower.isBusy()) {
                    // Switch to teleop mode for auto-align
                    enterTeleOpMode();
                    stateTimer.resetTimer();
                    state = State.ALIGN_AND_SPINUP;
                }
                break;

            case ALIGN_AND_SPINUP: {
                updateShooterFromTag();
                runAutoAlign();

                boolean shooterReady = sensorState.isShooterReady();
                boolean aligned = isAligned();
                boolean timedOut = stateTimer.getElapsedTimeSeconds() * 1000 >= ALIGN_TIMEOUT_MS;

                // Shoot once both ready, or on timeout if shooter is at least spinning
                if ((shooterReady && aligned) || timedOut) {
                    issueShootSequence();
                    stateTimer.resetTimer();
                    state = State.SHOOTING;
                }
                break;
            }

            case SHOOTING:
                updateShooterFromTag();
                runAutoAlign();
                if (isShootingDone()) {
                    sensorState.setShooterTargetVelocity(0);
                    exitTeleOpMode();
                    shootCycle++;

                    if (shootCycle > 2) {
                        state = State.DONE;
                    } else {
                        mechanismThread.enqueueCommand(
                                new MechanismThread.Command(
                                        MechanismThread.Command.Type.SET_AUTO_INDEX, true));
                        // Use curved path from first shoot to ball area row 1
                        if (intakeCycle == 0) {
                            driveCurvedToBallArea(ballAreaPoses[intakeCycle]);
                        } else {
                            driveToPose(ballAreaPoses[intakeCycle]);
                        }
                        state = State.DRIVE_TO_BALL_AREA;
                    }
                }
                break;

            // ==================== INTAKE PHASE ====================

            case DRIVE_TO_BALL_AREA:
                if (!follower.isBusy()) {
                    stateTimer.resetTimer();
                    state = State.SETTLE_AT_BALL_AREA;
                }
                break;

            case SETTLE_AT_BALL_AREA:
                if (stateTimer.getElapsedTimeSeconds() * 1000 >= BALL_AREA_SETTLE_DELAY_MS) {
                    currentBall = 0;
                    mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);
                    stateTimer.resetTimer();
                    state = State.SPIN_UP_INTAKE;
                }
                break;

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
                    mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);
                    advanceToNextBall();
                }
                break;
            }

            case WAIT_AUTO_INDEX: {
                ShootSequence.BallColor intakeColor =
                        sensorState.getPositionColor(SensorState.POS_INTAKE);
                boolean intakeCleared = (intakeColor == ShootSequence.BallColor.EMPTY);
                boolean isFull = (countBalls() >= 3);
                boolean mechIdle = mechanismThread.getStateDebug().contains("IDLE");

                if (intakeCleared || isFull
                        || (mechIdle && stateTimer.getElapsedTimeSeconds() > 0.5)) {
                    mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);
                    stateTimer.resetTimer();
                    state = State.PAUSE_BETWEEN;
                }

                if (stateTimer.getElapsedTimeSeconds() * 1000 >= 3000) {
                    mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);
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

            case DONE:
                break;
        }

        // Update follower AFTER state logic so teleop drive inputs are applied this frame
        follower.update();

        // ======================== TELEMETRY ========================
        panelsTelemetry.debug("Tag Range",
                sensorState.isBasketTagVisible()
                        ? String.format("%.1f in", sensorState.getTagRange()) : "n/a");
        panelsTelemetry.debug("Shooter Target",
                String.format("%.0f", sensorState.getShooterTargetVelocity()));
        panelsTelemetry.debug("Bearing Error",
                sensorState.isBasketTagVisible()
                        ? String.format("%.1f°", sensorState.getTargetBearing()) : "n/a");
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

    // ======================== AUTO-ALIGN ========================

    /** Switch follower to teleop drive mode and reset PID state. */
    private void enterTeleOpMode() {
        follower.startTeleOpDrive();
        resetPID();
        inTeleOpMode = true;
    }

    /** Exit teleop mode. Next driveToPose() call will resume path following. */
    private void exitTeleOpMode() {
        // Stop any residual drive
        follower.setTeleOpDrive(0, 0, 0, false);
        inTeleOpMode = false;
    }

    /**
     * Run one iteration of the auto-align PID.
     * Uses the same PID logic and constants as DriveThread.
     * Robot stays stationary (forward=0, strafe=0) and only rotates.
     */
    private void runAutoAlign() {
        if (!sensorState.isBasketTagVisible()) {
            // No tag — hold still
            follower.setTeleOpDrive(0, 0, 0, false);
            return;
        }

        double error = sensorState.getTargetBearing();
        double rotate = calculatePID(error);
        follower.setTeleOpDrive(0, 0, rotate, false);
    }

    /** Is the robot aligned to the target within the deadband? */
    private boolean isAligned() {
        if (!sensorState.isBasketTagVisible()) return false;
        return Math.abs(sensorState.getTargetBearing()) < SensorState.ALIGN_DEADBAND;
    }

    private double calculatePID(double error) {
        double dt = pidTimer.seconds();
        pidTimer.reset();

        if (Math.abs(error) < SensorState.ALIGN_DEADBAND) {
            integralSum = 0;
            lastError = error;
            return 0.0;
        }

        double p = SensorState.ALIGN_P * error;

        if (dt > 0 && dt < 1.0) {
            integralSum += error * dt;
            integralSum = clamp(integralSum, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        }
        double i = SensorState.ALIGN_I * integralSum;

        double d = 0;
        if (hasLastError && dt > 0 && dt < 1.0) {
            d = SensorState.ALIGN_D * (error - lastError) / dt;
        }

        lastError = error;
        hasLastError = true;

        return clamp(p + i + d, OUTPUT_MIN, OUTPUT_MAX);
    }

    private void resetPID() {
        integralSum = 0;
        lastError = 0;
        hasLastError = false;
        pidTimer.reset();
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    // ======================== SHOOT HELPERS ========================

    private void issueShootSequence() {
        mechanismThread.enqueueCommand(
                new MechanismThread.Command(
                        MechanismThread.Command.Type.SET_AUTO_INDEX, false));
        mechanismThread.enqueueCommand(
                new MechanismThread.Command(
                        MechanismThread.Command.Type.SHOOT_SEQUENCE, shootOrder.clone()));
    }

    private boolean isShootingDone() {
        boolean mechIdle = mechanismThread.getStateDebug().contains("IDLE");
        boolean startupGrace = stateTimer.getElapsedTimeSeconds() < 0.5;
        boolean timedOut = stateTimer.getElapsedTimeSeconds() * 1000 >= SHOOT_SEQUENCE_TIMEOUT_MS;
        return (!startupGrace && mechIdle) || timedOut;
    }

    private void updateShooterFromTag() {
        if (sensorState.isBasketTagVisible()) {
            sensorState.setVelocityFromDistance(sensorState.getTagRange());
        }
    }

    // ======================== INTAKE HELPERS ========================

    private void driveToPose(Pose target) {
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), target))
                .setLinearHeadingInterpolation(
                        follower.getPose().getHeading(), target.getHeading())
                .build();
        follower.followPath(path, true);
    }

    /**
     * Drive from startPose to the first shoot pose using a curved path
     * through START_TO_SHOOT0_CONTROL.
     */
    private void driveCurvedToFirstShoot(Pose target) {
        PathChain path = follower.pathBuilder()
                .addPath(new BezierCurve(
                        follower.getPose(),
                        START_TO_SHOOT0_CONTROL,
                        target))
                .setLinearHeadingInterpolation(
                        follower.getPose().getHeading(), target.getHeading())
                .build();
        follower.followPath(path, true);
    }

    /**
     * Drive from the end of intake row 1 to the shoot pose using a curved path.
     * The curve avoids obstacles by routing through a control point.
     */
    private void driveCurvedToShoot(Pose target) {
        PathChain path = follower.pathBuilder()
                .addPath(new BezierCurve(
                        follower.getPose(),
                        ROW1_TO_SHOOT_CONTROL,
                        target))
                .setLinearHeadingInterpolation(
                        follower.getPose().getHeading(), Math.toRadians(115))
                .build();
        follower.followPath(path, true);
    }

    /**
     * Drive from the first shoot position to ball area row 1 using a curved path.
     */
    private void driveCurvedToBallArea(Pose target) {
        PathChain path = follower.pathBuilder()
                .addPath(new BezierCurve(
                        follower.getPose(),
                        SHOOT1_TO_ROW1_CONTROL,
                        target))
                .setLinearHeadingInterpolation(
                        follower.getPose().getHeading(), target.getHeading())
                .build();
        follower.followPath(path, true);
    }

    private void driveToBall(int index) {
        Pose target = ballPoseRows[intakeCycle][index];
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), target))
                .setLinearHeadingInterpolation(
                        follower.getPose().getHeading(), target.getHeading())
                .build();
        follower.followPath(path, true);
    }

    private boolean checkBallDetected() {
        ShootSequence.BallColor c = sensorState.getPositionColor(SensorState.POS_INTAKE);
        return c == ShootSequence.BallColor.GREEN || c == ShootSequence.BallColor.PURPLE;
    }

    private void advanceToNextBall() {
        currentBall++;
        if (currentBall >= 3 || countBalls() >= 3) {
            mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
            sensorState.setShooterTargetVelocity(DEFAULT_SHOOTER_VELOCITY);

            // Use curved path when transitioning from intake row 1 → shoot pose
            if (intakeCycle == 0) {
                driveCurvedToShoot(shootPoses[shootCycle]);
            } else {
                driveToPose(shootPoses[shootCycle]);
            }

            intakeCycle++;
            state = State.DRIVE_TO_SHOOT;
            return;
        }

        // Always ensure intake is running and give it time to spin up before driving
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