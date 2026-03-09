package org.firstinspires.ftc.teamcode.threaded.Auto;

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
import org.firstinspires.ftc.teamcode.threaded.CameraThread;
import org.firstinspires.ftc.teamcode.threaded.ControlHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.ExpansionHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.MechanismThread;
import org.firstinspires.ftc.teamcode.threaded.SensorState;
import org.firstinspires.ftc.teamcode.threaded.ShooterThread;
import org.firstinspires.ftc.teamcode.threaded.ShootSequence;

@Autonomous(name = "Blue Far", group = "Auto")
@Configurable
public class Blue9Far extends OpMode {

    private static final int BASKET_TAG_ID = CameraThread.TAG_BLUE_BASKET;

    private static final ShootSequence.BallColor[] DEFAULT_SHOOT_ORDER = {
            ShootSequence.BallColor.GREEN,
            ShootSequence.BallColor.PURPLE,
            ShootSequence.BallColor.PURPLE
    };

    private final Pose startPose = new Pose(42.195, 9.187, Math.toRadians(90));

    public static class Paths {
        public PathChain initialShoot;
        public PathChain GoToBall1;
        public PathChain ball3;
        public PathChain shoot2;
        public PathChain gotoball3;
        public PathChain ball6;
        public PathChain shoot3;
        public PathChain park;

        public Paths(Follower follower) {
            initialShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.195, 9.187),
                                    new Pose(55.053, 19.475)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(112))
                    .build();

            GoToBall1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(55.053, 19.475),
                                    new Pose(45.289, 26.904),
                                    new Pose(43.641, 35.064)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(112), Math.toRadians(180))
                    .build();

            ball3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(43.641, 35.064),
                                    new Pose(16.291, 35.391)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            shoot2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(16.291, 35.391),
                                    new Pose(39.975, 34.362),
                                    new Pose(56.177, 17.415)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                    .build();

            gotoball3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.177, 17.415),
                                    new Pose(37.609, 27.778),
                                    new Pose(13, 26)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            ball6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(13, 26),
                                    new Pose(13, 6)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-155))
                    .build();

            shoot3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(10.861, 6.928),
                                    new Pose(27.744, 32.362),
                                    new Pose(50.237, 19.662)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-155), Math.toRadians(115))
                    .build();

            park = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(50.237, 19.662),
                                    new Pose(21.73211963589076, 38.99609882964888)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();
        }
    }

    public static double DEFAULT_SHOOTER_VELOCITY  = 130;
    public static long   BALL_AREA_SETTLE_DELAY_MS = 300;
    public static long   SHOOT_SEQUENCE_TIMEOUT_MS = 15000;
    public static long   ALIGN_TIMEOUT_MS          = 700;
    public static long   BALL_LINGER_TIMEOUT_MS    = 600;
    public static double COLLECT_MAX_POWER = .3;
    public static long   INTAKE_LINGER_DRIVING_MS = 1000;
    public static long   BALL6_COLLECT_TIMEOUT_MS  = 4000;

    private enum State {
        TAG_READING_AND_DRIVE,
        DRIVE_TO_SHOOT,
        ALIGN_AND_SPINUP,
        SHOOTING,
        DRIVE_TO_BALL_AREA,
        SETTLE_AT_BALL_AREA,
        COLLECTING,
        HOLDING,
        LINGER_AT_END,
        EJECTING,
        PARKING,
        DONE
    }

    private State state = State.TAG_READING_AND_DRIVE;

    private int shootCycle  = 0;
    private int intakeCycle = 0;
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
    private Timer collectTimer;
    private TelemetryManager panelsTelemetry;

    private boolean intakeLingeringDuringDrive = false;

    private double integralSum  = 0;
    private double lastError    = 0;
    private boolean hasLastError = false;
    private final ElapsedTime pidTimer = new ElapsedTime();
    private final ElapsedTime fullDebounceTimer = new ElapsedTime();
    private boolean fullDebouncing = false;
    public static long FULL_DEBOUNCE_MS = 500;

    private static final double INTEGRAL_LIMIT = 0.3;
    private static final double OUTPUT_MIN     = -1.0;
    private static final double OUTPUT_MAX     =  1.0;

    @Override
    public void init() {
        panelsTelemetry  = PanelsTelemetry.INSTANCE.getTelemetry();
        stateTimer       = new Timer();
        opmodeTimer      = new Timer();
        intakeLingerTimer = new Timer();
        collectTimer     = new Timer();

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

        shootCycle  = 0;
        intakeCycle = 0;
        inTeleOpMode = false;
        intakeLingeringDuringDrive = false;
        shootOrder = null;

        setFullSpeed();
        follower.followPath(paths.initialShoot, true);
        state = State.TAG_READING_AND_DRIVE;
    }

    @Override
    public void loop() {
        mechanismThread.setBallPositions(sensorState.getAllPositions());
        mechanismThread.enqueueCommand(
                new MechanismThread.Command(MechanismThread.Command.Type.SHOW_LIGHTS));

        boolean rampTriggered = sensorState.isRampTriggered();
        boolean mechIdle      = mechanismThread.isIdle();
        boolean rawFull = countBalls() >= 3;
        if (rawFull) {
            if (!fullDebouncing) { fullDebouncing = true; fullDebounceTimer.reset(); }
        } else {
            fullDebouncing = false;
        }
        boolean full = fullDebouncing && fullDebounceTimer.milliseconds() >= FULL_DEBOUNCE_MS;

        switch (state) {

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
                        setFullSpeed();
                        follower.followPath(paths.park, true);
                        state = State.PARKING;
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
                    collectTimer.resetTimer();
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
                    finishCollection();
                    break;
                }

                if (intakeCycle == 1
                        && collectTimer.getElapsedTimeSeconds() * 1000 >= BALL6_COLLECT_TIMEOUT_MS) {
                    follower.breakFollowing();
                    finishCollection();
                    break;
                }

                mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);

                if (rampTriggered && !mechIdle) {
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
                    finishCollection();
                    break;
                }

                if (intakeCycle == 1
                        && collectTimer.getElapsedTimeSeconds() * 1000 >= BALL6_COLLECT_TIMEOUT_MS) {
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

            case HOLDING:
                if (full && rampTriggered) {
                    state = State.EJECTING;
                    break;
                }
                if (full) {
                    finishCollection();
                    break;
                }

                if (intakeCycle == 1
                        && collectTimer.getElapsedTimeSeconds() * 1000 >= BALL6_COLLECT_TIMEOUT_MS) {
                    finishCollection();
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
                    finishCollection();
                }
                break;

            case PARKING:
                if (!follower.isBusy()) {
                    state = State.DONE;
                }
                break;

            case DONE:
                break;
        }

        follower.update();

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

    private void setFullSpeed()    { follower.setMaxPower(1.0); }
    private void setCollectSpeed() { follower.setMaxPower(COLLECT_MAX_POWER); }

    private PathChain getBallAreaPath() {
        return intakeCycle == 0 ? paths.GoToBall1 : paths.gotoball3;
    }

    private PathChain getCollectionPath() {
        return intakeCycle == 0 ? paths.ball3 : paths.ball6;
    }

    private PathChain getShootPath() {
        return shootCycle == 1 ? paths.shoot2 : paths.shoot3;
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

    private void issueShootSequence() {
        mechanismThread.enqueueCommand(
                new MechanismThread.Command(
                        MechanismThread.Command.Type.SET_AUTO_INDEX, false));
        mechanismThread.enqueueCommand(
                new MechanismThread.Command(
                        MechanismThread.Command.Type.SHOOT_SEQUENCE, shootOrder.clone()));
    }

    private boolean isShootingDone() {
        boolean mechIdle     = mechanismThread.isIdle();
        boolean startupGrace = stateTimer.getElapsedTimeSeconds() < 0.5;
        boolean timedOut     = stateTimer.getElapsedTimeSeconds() * 1000 >= SHOOT_SEQUENCE_TIMEOUT_MS;
        return (!startupGrace && mechIdle) || timedOut;
    }

    private void updateShooterFromTag() {
        if (sensorState.isBasketTagVisible()) {
            sensorState.setVelocityFromDistance(sensorState.getTagRange());
        }
    }

    /**
     * Count balls across all carousel slots.
     * Treats UNKNOWN as occupied — if a sensor reads UNKNOWN, something is
     * physically in that slot even if color classification failed.
     * Consistent with MechanismThread.isFull().
     */
    private int countBalls() {
        int count = 0;
        for (ShootSequence.BallColor c : sensorState.getAllPositions()) {
            if (c != ShootSequence.BallColor.EMPTY) count++;
        }
        return count;
    }

    private String shortColor(ShootSequence.BallColor c) {
        if (c == ShootSequence.BallColor.GREEN)  return "G";
        if (c == ShootSequence.BallColor.PURPLE) return "P";
        return "-";
    }
}