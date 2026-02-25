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
 * FULL AUTO (Blue Alliance)
 *
 * Flow:
 *   ReadTagAndGoToShoot: curved path, reads shoot order tag, ends at shoot position
 *   Shoot 0:  auto-align → shoot 3
 *   Intake 0: GoToBall1Position → Ball3 (single continuous collection path)
 *   Shoot 1:  Shoot2 → auto-align → shoot 3
 *   Intake 1: GoToBall4 → Ball6 (single continuous collection path)
 *   Shoot 2:  GoShoot3 → auto-align → shoot 3
 *   Done.
 *
 * Collection paths (Ball3, Ball6) use ramp-sensor-driven intake logic:
 *   - Intake runs continuously while driving slowly (0.5 max power).
 *   - If a ball hits the ramp while the carousel is still indexing the previous
 *     ball, drive pauses and intake stops — wheel holds the new ball in place.
 *   - Once the carousel is idle, drive and intake resume automatically.
 *   - If 3 balls are collected at any point, collection ends immediately.
 *
 * All non-collection paths run at full power.
 */
@Autonomous(name = "Blue 9 Ball Auto", group = "Auto")
@Configurable
public class BlueNew9BallAuto extends OpMode {

    // ======================== ALLIANCE CONFIG ========================

    private static final int BASKET_TAG_ID = CameraThread.TAG_BLUE_BASKET;

    private static final ShootSequence.BallColor[] DEFAULT_SHOOT_ORDER = {
            ShootSequence.BallColor.GREEN,
            ShootSequence.BallColor.PURPLE,
            ShootSequence.BallColor.PURPLE
    };

    // ======================== ROUTE POSES ========================

    private final Pose startPose = new Pose(27.964, 128.446, Math.toRadians(0));

    // ======================== PRE-BUILT PATHS ========================

    public static class Paths {
        public PathChain ReadTagAndGoToShoot;
        public PathChain GoToBall1Position;
        public PathChain Ball3;
        public PathChain Shoot2;
        public PathChain GoToBall4;
        public PathChain Ball6;
        public PathChain GoShoot3;

        public Paths(Follower follower) {
            ReadTagAndGoToShoot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(27.964, 128.446),
                                    new Pose(32.436, 102.731),
                                    new Pose(54.000, 89.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                    .build();

            GoToBall1Position = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(54.000, 89.000),
                                    new Pose(59.689, 66.841),
                                    new Pose(48.175, 60.183)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            Ball3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.175, 60.183),
                                    new Pose(9.888, 59.809)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            Shoot2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(9.888, 59.809),
                                    new Pose(45.000, 44.000),
                                    new Pose(58.000, 83.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                    .build();

            GoToBall4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.000, 83.000),
                                    new Pose(48.689, 84.311)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                    .build();

            Ball6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.689, 84.311),
                                    new Pose(19.139, 83.749)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            GoShoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.139, 83.749),
                                    new Pose(58.000, 83.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                    .build();
        }
    }

    // ======================== TUNABLE CONSTANTS ========================

    public static double DEFAULT_SHOOTER_VELOCITY  = 130;
    public static long   BALL_AREA_SETTLE_DELAY_MS = 300;
    public static long   SHOOT_SEQUENCE_TIMEOUT_MS = 15000;
    public static long   ALIGN_TIMEOUT_MS          = 700;
    public static long   BALL_LINGER_TIMEOUT_MS    = 600;

    /** Max power during collection paths (Ball3, Ball6). */
    public static double COLLECT_MAX_POWER = 0.5;

    /** How long to keep intake running at the start of DRIVE_TO_SHOOT (ms). */
    public static long INTAKE_LINGER_DRIVING_MS = 1000;

    // ======================== STATE MACHINE ========================

    private enum State {
        // --- Tag reading + first drive ---
        TAG_READING_AND_DRIVE,

        // --- Shoot phase (all 3 shoots) ---
        DRIVE_TO_SHOOT,
        ALIGN_AND_SPINUP,
        SHOOTING,

        // --- Intake phase (both intake cycles) ---
        DRIVE_TO_BALL_AREA,     // Approach path — full speed, intake on
        SETTLE_AT_BALL_AREA,    // Brief pause before starting collection
        COLLECTING,             // Slow path, intake on, watching ramp sensor
        HOLDING,                // Ramp fired while carousel busy: pause drive + intake
        LINGER_AT_END,          // Path ended, holding position briefly for last ball

        DONE
    }

    private State state = State.TAG_READING_AND_DRIVE;

    // Cycle tracking
    private int shootCycle  = 0;   // 0 = first shoot, 1 = second, 2 = third
    private int intakeCycle = 0;   // 0 = Ball3 row, 1 = Ball6 row

    // Whether we're in teleop drive mode (auto-align active)
    private boolean inTeleOpMode = false;

    private ShootSequence.BallColor[] shootOrder = null;

    private Follower              follower;
    private Paths                 paths;
    private MechanismThread       mechanismThread;
    private SensorState           sensorState;
    private CameraThread          cameraThread;
    private ShooterThread         shooterThread;
    private ControlHubI2CThread   controlHubI2C;
    private ExpansionHubI2CThread expansionHubI2C;

    private Timer stateTimer;
    private Timer opmodeTimer;
    private Timer intakeLingerTimer;
    private TelemetryManager panelsTelemetry;

    private boolean intakeLingeringDuringDrive = false;

    // ======================== AUTO-ALIGN PID ========================

    private double integralSum  = 0;
    private double lastError    = 0;
    private boolean hasLastError = false;
    private final ElapsedTime pidTimer = new ElapsedTime();

    private static final double INTEGRAL_LIMIT = 0.3;
    private static final double OUTPUT_MIN     = -1.0;
    private static final double OUTPUT_MAX     =  1.0;

    // ======================== LIFECYCLE ========================

    @Override
    public void init() {
        panelsTelemetry  = PanelsTelemetry.INSTANCE.getTelemetry();
        stateTimer       = new Timer();
        opmodeTimer      = new Timer();
        intakeLingerTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        paths = new Paths(follower);

        sensorState     = new SensorState(SensorState.Alliance.BLUE);
        mechanismThread = new MechanismThread(hardwareMap);
        mechanismThread.setSensorState(sensorState);
        controlHubI2C   = new ControlHubI2CThread(sensorState, hardwareMap);
        expansionHubI2C = new ExpansionHubI2CThread(sensorState, hardwareMap);
        shooterThread   = new ShooterThread(sensorState, hardwareMap);

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

        shootCycle  = 0;
        intakeCycle = 0;
        inTeleOpMode = false;
        intakeLingeringDuringDrive = false;
        shootOrder = null;

        setFullSpeed();
        follower.followPath(paths.ReadTagAndGoToShoot, true);
        state = State.TAG_READING_AND_DRIVE;
    }

    @Override
    public void loop() {
        mechanismThread.setBallPositions(sensorState.getAllPositions());
        mechanismThread.enqueueCommand(
                new MechanismThread.Command(MechanismThread.Command.Type.SHOW_LIGHTS));

        boolean rampTriggered = sensorState.isRampTriggered();
        boolean mechIdle      = mechanismThread.isIdle();
        boolean full          = countBalls() >= 3;

        switch (state) {

            // ==================== TAG READING + FIRST SHOOT DRIVE ====================

            case TAG_READING_AND_DRIVE:
                updateShooterFromTag();
                if (!follower.isBusy()) {
                    shootOrder = sensorState.hasDetectedShootOrder()
                            ? sensorState.getDetectedShootOrder()
                            : DEFAULT_SHOOT_ORDER.clone();
                    enterTeleOpMode();
                    stateTimer.resetTimer();
                    state = State.ALIGN_AND_SPINUP;
                }
                break;

            // ==================== SHOOT PHASE ====================

            case DRIVE_TO_SHOOT:
                updateShooterFromTag();

                if (intakeLingeringDuringDrive) {
                    mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);
                    if (intakeLingerTimer.getElapsedTimeSeconds() * 1000 >= INTAKE_LINGER_DRIVING_MS) {
                        mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
                        intakeLingeringDuringDrive = false;
                    }
                }

                if (!follower.isBusy()) {
                    mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
                    intakeLingeringDuringDrive = false;
                    enterTeleOpMode();
                    stateTimer.resetTimer();
                    state = State.ALIGN_AND_SPINUP;
                }
                break;

            case ALIGN_AND_SPINUP: {
                updateShooterFromTag();
                runAutoAlign();

                boolean shooterReady = sensorState.isShooterReady();
                boolean aligned      = isAligned();
                boolean timedOut     = stateTimer.getElapsedTimeSeconds() * 1000 >= ALIGN_TIMEOUT_MS;

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
                        mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);
                        setFullSpeed();
                        follower.followPath(getBallAreaPath(), true);
                        state = State.DRIVE_TO_BALL_AREA;
                    }
                }
                break;

            // ==================== INTAKE PHASE ====================

            // Approach path — full speed, intake already on from SHOOTING exit
            case DRIVE_TO_BALL_AREA:
                mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);
                if (!follower.isBusy()) {
                    stateTimer.resetTimer();
                    state = State.SETTLE_AT_BALL_AREA;
                }
                break;

            // Brief pause to let the robot settle before collection starts
            case SETTLE_AT_BALL_AREA:
                mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);
                if (stateTimer.getElapsedTimeSeconds() * 1000 >= BALL_AREA_SETTLE_DELAY_MS) {
                    setCollectSpeed();
                    follower.followPath(getCollectionPath(), true);
                    state = State.COLLECTING;
                }
                break;

            // Slow path — intake on, watching ramp sensor.
            // Ramp fires while carousel busy → pause drive + stop intake (HOLDING).
            // Path ends or full → done collecting.
            case COLLECTING:
                if (full) {
                    finishCollection();
                    break;
                }

                mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);

                if (rampTriggered && !mechIdle) {
                    // New ball committed to ramp, carousel still indexing — hold it
                    mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
                    follower.breakFollowing();
                    state = State.HOLDING;
                    break;
                }

                if (!follower.isBusy()) {
                    // End of collection path — linger briefly in case a ball is mid-ramp
                    stateTimer.resetTimer();
                    state = State.LINGER_AT_END;
                }
                break;

            // Path ended. Hold still with intake on for BALL_LINGER_TIMEOUT_MS in case
            // a ball is still rolling in, then finish regardless.
            case LINGER_AT_END:
                if (full) {
                    finishCollection();
                    break;
                }

                mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);

                if (rampTriggered && !mechIdle) {
                    mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
                    state = State.HOLDING;
                    break;
                }

                if (stateTimer.getElapsedTimeSeconds() * 1000 >= BALL_LINGER_TIMEOUT_MS) {
                    finishCollection();
                }
                break;

            // Carousel busy — intake stopped, ball held on ramp by still wheel.
            // Resume as soon as carousel is idle and there's room for the ball.
            case HOLDING:
                mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);

                if (full) {
                    finishCollection();
                    break;
                }

                if (mechIdle) {
                    // Carousel done — resume drive and intake
                    setCollectSpeed();
                    follower.followPath(getCollectionPath(), true);
                    mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);
                    state = State.COLLECTING;
                }
                break;

            case DONE:
                break;
        }

        follower.update();

        // ======================== TELEMETRY ========================
        panelsTelemetry.debug("State",        state.name());
        panelsTelemetry.debug("Shoot Cycle",  String.valueOf(shootCycle));
        panelsTelemetry.debug("Intake Cycle", String.valueOf(intakeCycle));
        panelsTelemetry.debug("Balls",        String.format("%d / 3", countBalls()));
        panelsTelemetry.debug("Ramp Sensor",  rampTriggered ? "TRIGGERED" : "clear");
        panelsTelemetry.debug("Mech",         mechIdle ? "IDLE" : mechanismThread.getStateDebug());
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

    // ======================== SPEED HELPERS ========================

    private void setFullSpeed()    { follower.setMaxPower(1.0); }
    private void setCollectSpeed() { follower.setMaxPower(COLLECT_MAX_POWER); }

    // ======================== PATH SELECTION ========================

    private PathChain getBallAreaPath() {
        return intakeCycle == 0 ? paths.GoToBall1Position : paths.GoToBall4;
    }

    private PathChain getCollectionPath() {
        return intakeCycle == 0 ? paths.Ball3 : paths.Ball6;
    }

    private PathChain getShootPath() {
        return shootCycle == 1 ? paths.Shoot2 : paths.GoShoot3;
    }

    // ======================== COLLECTION FINISH ========================

    /**
     * Called when collection is complete (path done, full, or linger timed out).
     * Stops intake, arms shooter, starts linger timer, drives to shoot position.
     */
    private void finishCollection() {
        sensorState.setShooterTargetVelocity(DEFAULT_SHOOTER_VELOCITY);
        intakeLingeringDuringDrive = true;
        intakeLingerTimer.resetTimer();
        setFullSpeed();
        follower.followPath(getShootPath(), true);
        intakeCycle++;
        state = State.DRIVE_TO_SHOOT;
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
        double rotate = calculatePID(sensorState.getTargetBearing());
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
        integralSum  = 0;
        lastError    = 0;
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
        boolean mechIdle    = mechanismThread.getStateDebug().contains("IDLE");
        boolean startupGrace = stateTimer.getElapsedTimeSeconds() < 0.5;
        boolean timedOut    = stateTimer.getElapsedTimeSeconds() * 1000 >= SHOOT_SEQUENCE_TIMEOUT_MS;
        return (!startupGrace && mechIdle) || timedOut;
    }

    private void updateShooterFromTag() {
        if (sensorState.isBasketTagVisible()) {
            sensorState.setVelocityFromDistance(sensorState.getTagRange());
        }
    }

    // ======================== HELPERS ========================

    private int countBalls() {
        int count = 0;
        for (ShootSequence.BallColor c : sensorState.getAllPositions()) {
            if (c == ShootSequence.BallColor.GREEN || c == ShootSequence.BallColor.PURPLE) count++;
        }
        return count;
    }

    private String shortColor(ShootSequence.BallColor c) {
        if (c == ShootSequence.BallColor.GREEN)  return "G";
        if (c == ShootSequence.BallColor.PURPLE) return "P";
        return "-";
    }
}