package org.firstinspires.ftc.teamcode.threaded.Old.Auto;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.threaded.Old.CameraThread;
import org.firstinspires.ftc.teamcode.threaded.Old.ControlHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.Old.ExpansionHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.Old.MechanismThread;
import org.firstinspires.ftc.teamcode.threaded.Old.SensorState;
import org.firstinspires.ftc.teamcode.threaded.Old.ShooterThread;
import org.firstinspires.ftc.teamcode.threaded.Old.ShootSequence;

/**
 * BLUE AUTO — Shoot preloaded, collect 4-5-6, shoot, collect 1-2-3, stop.
 *
 * Flow:
 *   ReadTagAndGoToShoot: Drive curved path while camera reads shoot order tag
 *   Shoot 0: auto-align → shoot 3 preloaded balls
 *   Intake 0: GoToBall4 → pick up Ball4, Ball5, Ball6
 *   Shoot 1: GoShoot3 → auto-align → shoot 3
 *   Intake 1: GoToBall1Position → pick up Ball1, Ball2, Ball3
 *   Done (stop in place with balls collected).
 */

@Disabled
@Autonomous(name = "Blue 6 Ball Auto", group = "Auto")
@Configurable
public class Blue6Ball456Auto extends OpMode {

    // ======================== ALLIANCE CONFIG ========================

    private static final int BASKET_TAG_ID = CameraThread.TAG_BLUE_BASKET;

    private static final ShootSequence.BallColor[] DEFAULT_SHOOT_ORDER = {
            ShootSequence.BallColor.GREEN,
            ShootSequence.BallColor.PURPLE,
            ShootSequence.BallColor.PURPLE
    };

    // ======================== ROUTE POSES ========================

    private final Pose startPose = new Pose(25.5, 129, Math.toRadians(0));

    // ======================== PRE-BUILT PATHS ========================

    public static class Paths {
        public PathChain ReadTagAndGoToShoot;
        public PathChain GoToBall1Position;
        public PathChain Ball1;
        public PathChain Ball2;
        public PathChain Ball3;
        public PathChain GoToBall4;
        public PathChain Ball4;
        public PathChain Ball5;
        public PathChain Ball6;
        public PathChain GoShoot3;
        public PathChain ShootTeleop;

        public Paths(Follower follower) {
            ReadTagAndGoToShoot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(25.5, 125),
                                    new Pose(47.70056176853055, 109.8610156046814),
                                    new Pose(58, 86)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(130))
                    .build();

            GoToBall1Position = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48, 90),
                                    new Pose(55.131339401820576, 75.6176853055917),
                                    new Pose(41.000, 63.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                    .build();

            Ball1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(41.000, 63.000),
                                    new Pose(35.000, 63.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            Ball2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(35.000, 63.000),
                                    new Pose(30, 63.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            Ball3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(30, 63.000),
                                    new Pose(20.000, 63.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            GoToBall4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.000, 86.000),
                                    new Pose(40.500, 86.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                    .build();

            Ball4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(40.500, 86.000),
                                    new Pose(36.500, 86.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            Ball5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(36.500, 86.000),
                                    new Pose(31.500, 86.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            Ball6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(31.500, 86.000),
                                    new Pose(21.000, 86.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            GoShoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(24.000, 86.000),
                                    new Pose(48.000, 90.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                    .build();

            ShootTeleop = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(20.000, 63.000),
                                    new Pose(53.255, 73.367),
                                    new Pose(53.500, 120.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(160))

                    .build();
        }
    }

    // ======================== TUNABLE TIMING ========================

    public static long BALL_LINGER_TIMEOUT_MS = 250;
    public static long PAUSE_AFTER_INDEX_MS = 200;
    public static double DEFAULT_SHOOTER_VELOCITY = 130;
    public static long BALL_AREA_SETTLE_DELAY_MS = 200;
    public static long SHOOTER_SPINUP_TIMEOUT_MS = 3000;
    public static long SHOOT_SEQUENCE_TIMEOUT_MS = 15000;

    /** Max time to wait for auto-align before shooting anyway (ms). */
    public static long ALIGN_TIMEOUT_MS = 700;

    // ======================== STATE MACHINE ========================

    private enum State {
        // --- Tag reading + drive to first shoot ---
        TAG_READING_AND_DRIVE,

        // --- Shoot phase ---
        DRIVE_TO_SHOOT,
        ALIGN_AND_SPINUP,
        SHOOTING,

        // --- Intake phase ---
        DRIVE_TO_BALL_AREA,
        SETTLE_AT_BALL_AREA,
        DRIVE_TO_BALL,
        LINGER_FOR_BALL,
        WAIT_AUTO_INDEX,
        PAUSE_BETWEEN,

        DONE
    }

    private State state = State.TAG_READING_AND_DRIVE;

    // Cycle tracking
    private int shootCycle = 0;     // 0 = first shoot (preloaded), 1 = second (after 4-5-6)
    private int intakeCycle = 0;    // 0 = balls 4-5-6, 1 = balls 1-2-3

    // Current intake ball index within a row (0, 1, 2)
    private int currentBall = 0;

    // Whether we're in teleop drive mode (auto-align active)
    private boolean inTeleOpMode = false;

    private ShootSequence.BallColor[] shootOrder = null;

    private Follower follower;
    private Paths paths;
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

        paths = new Paths(follower);

        sensorState = new SensorState(SensorState.Alliance.BLUE);
        mechanismThread = new MechanismThread(hardwareMap);
        mechanismThread.setSensorState(sensorState);
        controlHubI2C = new ControlHubI2CThread(sensorState, hardwareMap);
        expansionHubI2C = new ExpansionHubI2CThread(sensorState, hardwareMap);
        shooterThread = new ShooterThread(sensorState, hardwareMap);

        cameraThread = new CameraThread(sensorState, hardwareMap, BASKET_TAG_ID);
        cameraThread.start();

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
        opmodeTimer.resetTimer();
        stateTimer.resetTimer();

        mechanismThread.start();
        controlHubI2C.start();
        expansionHubI2C.start();
        shooterThread.start();

        sensorState.setShooterTargetVelocity(DEFAULT_SHOOTER_VELOCITY);

        shootCycle = 0;
        intakeCycle = 0;
        inTeleOpMode = false;

        shootOrder = null;

        follower.followPath(paths.ReadTagAndGoToShoot, true);
        state = State.TAG_READING_AND_DRIVE;
    }

    @Override
    public void loop() {
        mechanismThread.setBallPositions(sensorState.getAllPositions());

        switch (state) {

            // ==================== TAG READING + FIRST SHOOT DRIVE ====================

            case TAG_READING_AND_DRIVE:
                updateShooterFromTag();
                if (!follower.isBusy()) {
                    if (sensorState.hasDetectedShootOrder()) {
                        shootOrder = sensorState.getDetectedShootOrder();
                    } else {
                        shootOrder = DEFAULT_SHOOT_ORDER.clone();
                    }

                    enterTeleOpMode();
                    stateTimer.resetTimer();
                    state = State.ALIGN_AND_SPINUP;
                }
                break;

            // ==================== SHOOT PHASE ====================

            case DRIVE_TO_SHOOT:
                updateShooterFromTag();
                if (!follower.isBusy()) {
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

                    // After each shoot, go collect next batch of balls
                    // intakeCycle 0 → balls 4-5-6, intakeCycle 1 → balls 1-2-3
                    mechanismThread.enqueueCommand(
                            new MechanismThread.Command(
                                    MechanismThread.Command.Type.SET_AUTO_INDEX, true));
                    follower.followPath(getBallAreaPath(), true);
                    state = State.DRIVE_TO_BALL_AREA;
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
                    follower.followPath(getBallPath(intakeCycle, currentBall), true);
                    state = State.DRIVE_TO_BALL;
                }
                break;

            case DRIVE_TO_BALL: {
                mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);

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
                mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);

                if (checkBallDetected()) {
                    stateTimer.resetTimer();
                    state = State.WAIT_AUTO_INDEX;
                    break;
                }
                if (stateTimer.getElapsedTimeSeconds() * 1000 >= BALL_LINGER_TIMEOUT_MS) {
                    advanceToNextBall();
                }
                break;
            }

            case WAIT_AUTO_INDEX: {
                mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);

                ShootSequence.BallColor intakeColor =
                        sensorState.getPositionColor(SensorState.POS_INTAKE);
                boolean intakeCleared = (intakeColor == ShootSequence.BallColor.EMPTY);
                boolean isFull = (countBalls() >= 3);
                boolean mechIdle = mechanismThread.getStateDebug().contains("IDLE");

                if (intakeCleared || isFull
                        || (mechIdle && stateTimer.getElapsedTimeSeconds() > 0.5)) {
                    stateTimer.resetTimer();
                    state = State.PAUSE_BETWEEN;
                }

                if (stateTimer.getElapsedTimeSeconds() * 1000 >= 3000) {
                    stateTimer.resetTimer();
                    state = State.PAUSE_BETWEEN;
                }
                break;
            }

            case PAUSE_BETWEEN:
                mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);

                if (stateTimer.getElapsedTimeSeconds() * 1000 >= PAUSE_AFTER_INDEX_MS) {
                    advanceToNextBall();
                }
                break;

            case DONE:
                break;
        }

        follower.update();

        // ======================== TELEMETRY ========================
        panelsTelemetry.debug("State", state.name());
        panelsTelemetry.debug("Shoot Cycle", String.valueOf(shootCycle));
        panelsTelemetry.debug("Intake Cycle", String.valueOf(intakeCycle));
        panelsTelemetry.debug("Current Ball", String.valueOf(currentBall));
        panelsTelemetry.debug("Shoot Order",
                shootOrder != null
                        ? shortColor(shootOrder[0]) + " " + shortColor(shootOrder[1]) + " " + shortColor(shootOrder[2])
                        : "not locked in");
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

    // ======================== PATH SELECTION ========================

    /**
     * Intake cycle 0 → balls 4-5-6 (GoToBall4)
     * Intake cycle 1 → balls 1-2-3 (GoToBall1Position)
     */
    private PathChain getBallAreaPath() {
        return intakeCycle == 0 ? paths.GoToBall4 : paths.GoToBall1Position;
    }

    /**
     * Only called for shootCycle 1 (after collecting 4-5-6).
     * GoShoot3 goes from near Ball6 end to the shoot position.
     */
    private PathChain getShootPath() {
        return paths.GoShoot3;
    }

    /**
     * Intake cycle 0 → Ball4, Ball5, Ball6
     * Intake cycle 1 → Ball1, Ball2, Ball3
     */
    private PathChain getBallPath(int cycle, int ball) {
        if (cycle == 0) {
            switch (ball) {
                case 0: return paths.Ball4;
                case 1: return paths.Ball5;
                case 2: return paths.Ball6;
            }
        } else {
            switch (ball) {
                case 0: return paths.Ball1;
                case 1: return paths.Ball2;
                case 2: return paths.Ball3;
            }
        }
        return paths.Ball4;
    }

    // ======================== AUTO-ALIGN ========================

    private void enterTeleOpMode() {
        follower.startTeleOpDrive();
        resetPID();
        inTeleOpMode = true;
    }

    private void exitTeleOpMode() {
        follower.setTeleOpDrive(0, 0, 0, false);
        inTeleOpMode = false;
    }

    private void runAutoAlign() {
        if (!sensorState.isBasketTagVisible()) {
            follower.setTeleOpDrive(0, 0, 0, false);
            return;
        }

        double error = sensorState.getTargetBearing();
        double rotate = calculatePID(error);
        follower.setTeleOpDrive(0, 0, rotate, false);
    }

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

    private boolean checkBallDetected() {
        ShootSequence.BallColor c = sensorState.getPositionColor(SensorState.POS_INTAKE);
        return c == ShootSequence.BallColor.GREEN || c == ShootSequence.BallColor.PURPLE;
    }

    private void advanceToNextBall() {
        currentBall++;
        if (currentBall >= 3 || countBalls() >= 3) {
            mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);

            // After second intake (balls 1-2-3), just stop
            if (intakeCycle >= 1) {
                intakeCycle++;
                state = State.DONE;
                return;
            }

            // After first intake (balls 4-5-6), go shoot
            sensorState.setShooterTargetVelocity(DEFAULT_SHOOTER_VELOCITY);
            follower.followPath(getShootPath(), true);
            intakeCycle++;
            state = State.DRIVE_TO_SHOOT;
            return;
        }

        follower.followPath(getBallPath(intakeCycle, currentBall), true);
        state = State.DRIVE_TO_BALL;
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