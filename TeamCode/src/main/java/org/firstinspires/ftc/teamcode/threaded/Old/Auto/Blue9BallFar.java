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
 * FULL AUTO (Blue Alliance) — Alternate Path Layout
 *
 * Flow:
 *   initialreadtagandshoot: drive to shoot position, reads shoot order tag
 *   Shoot 0:  auto-align → shoot 3
 *   Intake 0: gotoball1position → ball3
 *   Shoot 1:  goshoot2 → auto-align → shoot 3
 *   Intake 1: gotoball4 → ball6 → goforwardabit  (two collection segments)
 *   Shoot 2:  goshoot3 → auto-align → shoot 3
 *   Done.
 *
 * collectionSegment tracks which collecting path we're on within a cycle.
 * Intake cycle 1 has two segments: ball6 (seg 0) then goforwardabit (seg 1).
 * When seg 0 ends and the robot isn't full, it rolls directly into seg 1.
 */
@Autonomous(name = "Blue 9 fa", group = "Auto")
@Configurable
public class Blue9BallFar extends OpMode {

    // ======================== ALLIANCE CONFIG ========================

    private static final int BASKET_TAG_ID = CameraThread.TAG_BLUE_BASKET;

    private static final ShootSequence.BallColor[] DEFAULT_SHOOT_ORDER = {
            ShootSequence.BallColor.GREEN,
            ShootSequence.BallColor.PURPLE,
            ShootSequence.BallColor.PURPLE
    };

    // ======================== ROUTE POSES ========================

    private final Pose startPose = new Pose(42.195, 9.187, Math.toRadians(90));

    // ======================== PRE-BUILT PATHS ========================

    public static class Paths {
        public PathChain initialreadtagandshoot;
        public PathChain gotoball1position;
        public PathChain ball3;
        public PathChain goshoot2;
        public PathChain gotoball4;
        public PathChain ball6;
        public PathChain goforwardabit;
        public PathChain goshoot3;

        public Paths(Follower follower) {
            initialreadtagandshoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.195, 9.187),
                                    new Pose(56.637, 17.259)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))
                    .build();

            gotoball1position = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.637, 17.259),
                                    new Pose(48.408, 25.125),
                                    new Pose(43.267, 35.438)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                    .build();

            ball3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(43.267, 35.438),
                                    new Pose(10.505, 35.638)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            goshoot2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(10.505, 35.638),
                                    new Pose(44.547, 33.375),
                                    new Pose(54.000, 17.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(108))
                    .build();

            gotoball4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(54.000, 17.000),
                                    new Pose(44.839, 45.030),
                                    new Pose(12.000, 24.187)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(230))
                    .build();

            ball6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.000, 24.187),
                                    new Pose(12.000, 9.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(230))
                    .build();

            goforwardabit = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.000, 9.500),
                                    new Pose(9.363, 8.239)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            goshoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(9.363, 8.239),
                                    new Pose(57.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                    .build();
        }
    }

    // ======================== TUNABLE CONSTANTS ========================

    public static double DEFAULT_SHOOTER_VELOCITY  = 130;
    public static long   BALL_AREA_SETTLE_DELAY_MS = 300;
    public static long   SHOOT_SEQUENCE_TIMEOUT_MS = 15000;
    public static long   ALIGN_TIMEOUT_MS          = 700;
    public static long   BALL_LINGER_TIMEOUT_MS    = 600;

    /** Max power during collection paths (ball3, ball6, goforwardabit). */
    public static double COLLECT_MAX_POWER = .3;

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
        EJECTING,               // Carousel full + ball on ramp: reverse intake until ramp clears

        DONE
    }

    private State state = State.TAG_READING_AND_DRIVE;

    // Cycle tracking
    private int shootCycle        = 0;  // 0 = first shoot, 1 = second, 2 = third
    private int intakeCycle       = 0;  // 0 = ball3 row, 1 = ball6/goforwardabit row
    private int collectionSegment = 0;  // within a cycle: 0 = first collect path, 1 = second (goforwardabit)

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
        mechanismThread.setSkipKickback(true);
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

        shootCycle        = 0;
        intakeCycle       = 0;
        collectionSegment = 0;
        inTeleOpMode      = false;
        intakeLingeringDuringDrive = false;
        shootOrder        = null;

        setFullSpeed();
        follower.followPath(paths.initialreadtagandshoot, true);
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
                        collectionSegment = 0;
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

            case DRIVE_TO_BALL_AREA:
                mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);
                if (!follower.isBusy()) {
                    stateTimer.resetTimer();
                    state = State.SETTLE_AT_BALL_AREA;
                }
                break;

            case SETTLE_AT_BALL_AREA:
                mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);
                if (stateTimer.getElapsedTimeSeconds() * 1000 >= BALL_AREA_SETTLE_DELAY_MS) {
                    setCollectSpeed();
                    follower.followPath(getCollectionPath(), true);
                    state = State.COLLECTING;
                }
                break;

            case COLLECTING:
                if (full && rampTriggered) {
                    follower.breakFollowing();
                    state = State.EJECTING;
                    break;
                }
                if (full) {
                    advanceCollectionOrFinish();
                    break;
                }

                mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);

                if (rampTriggered && !mechIdle && intakeSlotOccupied()) {
                    mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
                    follower.breakFollowing();
                    state = State.HOLDING;
                    break;
                }

                if (!follower.isBusy()) {
                    stateTimer.resetTimer();
                    state = State.LINGER_AT_END;
                }
                break;

            case LINGER_AT_END:
                if (full && rampTriggered) {
                    state = State.EJECTING;
                    break;
                }
                if (full) {
                    advanceCollectionOrFinish();
                    break;
                }

                mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);

                if (rampTriggered && !mechIdle && intakeSlotOccupied()) {
                    mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
                    state = State.HOLDING;
                    break;
                }

                if (stateTimer.getElapsedTimeSeconds() * 1000 >= BALL_LINGER_TIMEOUT_MS) {
                    advanceCollectionOrFinish();
                }
                break;

            case HOLDING:
                if (full && rampTriggered) {
                    state = State.EJECTING;
                    break;
                }
                if (full) {
                    advanceCollectionOrFinish();
                    break;
                }

                mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);

                if (mechIdle) {
                    setCollectSpeed();
                    follower.followPath(getCollectionPath(), true);
                    mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);
                    state = State.COLLECTING;
                }
                break;

            case EJECTING:
                mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.OUT);

                if (!rampTriggered) {
                    mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
                    advanceCollectionOrFinish();
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
        panelsTelemetry.debug("Collect Seg",  String.valueOf(collectionSegment));
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

    /** Approach (full-speed) path for the current intake cycle. */
    private PathChain getBallAreaPath() {
        return intakeCycle == 0 ? paths.gotoball1position : paths.gotoball4;
    }

    /**
     * Current collection (slow) path.
     * Cycle 0:  seg 0 → ball3
     * Cycle 1:  seg 0 → ball6,  seg 1 → goforwardabit
     */
    private PathChain getCollectionPath() {
        if (intakeCycle == 0) return paths.ball3;
        return collectionSegment == 0 ? paths.ball6 : paths.goforwardabit;
    }

    /** True when a second collection segment exists and hasn't been run yet. */
    private boolean hasNextCollectionSegment() {
        return intakeCycle == 1 && collectionSegment == 0;
    }

    private PathChain getShootPath() {
        return shootCycle == 1 ? paths.goshoot2 : paths.goshoot3;
    }

    // ======================== COLLECTION ADVANCE / FINISH ========================

    /**
     * Called whenever a collection phase ends (path done + linger, full, or ejected).
     * If a second collection segment is still available (goforwardabit), rolls into it.
     * Otherwise, arms the shooter and drives to the shoot position.
     */
    private void advanceCollectionOrFinish() {
        if (hasNextCollectionSegment()) {
            collectionSegment = 1;
            setCollectSpeed();
            follower.followPath(getCollectionPath(), true);
            state = State.COLLECTING;
        } else {
            finishCollection();
        }
    }

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
        boolean mechIdle     = mechanismThread.getStateDebug().contains("IDLE");
        boolean startupGrace = stateTimer.getElapsedTimeSeconds() < 0.5;
        boolean timedOut     = stateTimer.getElapsedTimeSeconds() * 1000 >= SHOOT_SEQUENCE_TIMEOUT_MS;
        return (!startupGrace && mechIdle) || timedOut;
    }

    private void updateShooterFromTag() {
        if (sensorState.isBasketTagVisible()) {
            sensorState.setVelocityFromDistance(sensorState.getTagRange());
        }
    }

    // ======================== HELPERS ========================

    private boolean intakeSlotOccupied() {
        ShootSequence.BallColor c = sensorState.getPositionColor(SensorState.POS_INTAKE);
        return c == ShootSequence.BallColor.GREEN || c == ShootSequence.BallColor.PURPLE;
    }

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