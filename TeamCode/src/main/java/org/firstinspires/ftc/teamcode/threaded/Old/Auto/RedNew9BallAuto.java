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

@Autonomous(name = "Red 9 Ball Auto", group = "Auto")
@Configurable
public class RedNew9BallAuto extends OpMode {

    private static final int BASKET_TAG_ID = CameraThread.TAG_RED_BASKET;

    private static final ShootSequence.BallColor[] DEFAULT_SHOOT_ORDER = {
            ShootSequence.BallColor.GREEN,
            ShootSequence.BallColor.PURPLE,
            ShootSequence.BallColor.PURPLE
    };

    // Blue start was (27.964, 128.446, 0°). Mirrored: X = 144 - 27.964 = 116.036, heading = 180°
    private final Pose startPose = new Pose(116.036, 128.446, Math.toRadians(180));

    public static class Paths {
        public PathChain ReadTagAndGoToShoot;
        public PathChain GoToBall1Position;
        public PathChain Ball3;
        public PathChain Shoot2;
        public PathChain GoToBall4;
        public PathChain Ball6;
        public PathChain GoShoot3;

        public Paths(Follower follower) {
            // Blue: (27.964,128.446) → (32.436,102.731) → (54.000,89.000), heading 0°→125°
            // Red:  X mirrored (144-X), heading π-θ → 180°→55°
            ReadTagAndGoToShoot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(116.036, 128.446),
                                    new Pose(111.564, 102.731),
                                    new Pose(93.000, 89.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(55))
                    .build();

            // Blue: (54.000,89.000) → (59.689,66.841) → (48.175,60.183), heading 135°→180°
            // Red:  heading 45°→0°
            GoToBall1Position = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(90.000, 89.000),
                                    new Pose(84.311, 66.841),
                                    new Pose(96.5, 60.183)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            // Blue: (48.175,60.183) → (9.888,59.809), tangent
            // Red:  tangent heading (direction is now rightward, tangent handles it)
            Ball3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.5, 60.183),
                                    new Pose(134.112, 59.809)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Blue: (9.888,59.809) → (45.000,44.000) → (58.000,83.000), heading 180°→140°
            // Red:  heading 0°→40°
            Shoot2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(134.112, 59.809),
                                    new Pose(99.000, 44.000),
                                    new Pose(90.000, 83.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                    .build();

            // Blue: (58.000,83.000) → (48.689,84.311), heading 140°→180°
            // Red:  heading 40°→0°
            GoToBall4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.000, 83.000),
                                    new Pose(95.311, 84.311)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                    .build();

            // Blue: (48.689,84.311) → (19.139,83.749), tangent
            Ball6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(95.311, 84.311),
                                    new Pose(124.861, 83.749)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Blue: (19.139,83.749) → (58,107), heading 180°→155°
            // Red:  heading 0°→25°
            GoShoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(124.861, 83.749),
                                    new Pose(90, 107)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(25))
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
    private TelemetryManager panelsTelemetry;

    private boolean intakeLingeringDuringDrive = false;

    private double integralSum  = 0;
    private double lastError    = 0;
    private boolean hasLastError = false;
    private final ElapsedTime pidTimer = new ElapsedTime();

    private static final double INTEGRAL_LIMIT = 0.3;
    private static final double OUTPUT_MIN     = -1.0;
    private static final double OUTPUT_MAX     =  1.0;

    @Override
    public void init() {
        panelsTelemetry  = PanelsTelemetry.INSTANCE.getTelemetry();
        stateTimer       = new Timer();
        opmodeTimer      = new Timer();
        intakeLingerTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        paths = new Paths(follower);

        sensorState     = new SensorState(SensorState.Alliance.RED);
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
        return intakeCycle == 0 ? paths.GoToBall1Position : paths.GoToBall4;
    }

    private PathChain getCollectionPath() {
        return intakeCycle == 0 ? paths.Ball3 : paths.Ball6;
    }

    private PathChain getShootPath() {
        return shootCycle == 1 ? paths.Shoot2 : paths.GoShoot3;
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