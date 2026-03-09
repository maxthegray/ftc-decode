package org.firstinspires.ftc.teamcode.threaded.Auto;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.threaded.CameraThread;
import org.firstinspires.ftc.teamcode.threaded.ControlHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.ExpansionHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.MechanismThread;
import org.firstinspires.ftc.teamcode.threaded.SensorState;
import org.firstinspires.ftc.teamcode.threaded.ShooterThread;
import org.firstinspires.ftc.teamcode.threaded.ShootSequence;

@Autonomous(name = "Blue Far Simple", group = "Auto")
@Configurable
public class Blue9FarSimple extends OpMode {

    private static final int BASKET_TAG_ID = CameraThread.TAG_BLUE_BASKET;

    private static final ShootSequence.BallColor[] DEFAULT_SHOOT_ORDER = {
            ShootSequence.BallColor.GREEN,
            ShootSequence.BallColor.PURPLE,
            ShootSequence.BallColor.PURPLE
    };

    private final Pose startPose = new Pose(42.195, 9.187, Math.toRadians(90));

    public static class Paths {
        public PathChain initialShoot;
        public PathChain park;

        public Paths(Follower follower) {
            initialShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.195, 9.187),
                                    new Pose(55.053, 19.475)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(112))
                    .build();

            park = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55.053, 19.475),
                                    new Pose(45.053, 19.475)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();
        }
    }

    public static double DEFAULT_SHOOTER_VELOCITY  = 130;
    public static long   SHOOT_SEQUENCE_TIMEOUT_MS = 15000;
    public static long   ALIGN_TIMEOUT_MS          = 700;

    private enum State {
        TAG_READING_AND_DRIVE,
        ALIGN_AND_SPINUP,
        SHOOTING,
        PARKING,
        DONE
    }

    private State state = State.TAG_READING_AND_DRIVE;

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
    private TelemetryManager panelsTelemetry;

    private double integralSum  = 0;
    private double lastError    = 0;
    private boolean hasLastError = false;
    private final ElapsedTime pidTimer = new ElapsedTime();

    private static final double INTEGRAL_LIMIT = 0.3;
    private static final double OUTPUT_MIN     = -1.0;
    private static final double OUTPUT_MAX     =  1.0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        stateTimer      = new Timer();

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
        stateTimer.resetTimer();

        mechanismThread.start();
        controlHubI2C.start();
        expansionHubI2C.start();
        shooterThread.start();

        sensorState.setShooterTargetVelocity(DEFAULT_SHOOTER_VELOCITY);
        shootOrder = null;

        follower.setMaxPower(1.0);
        follower.followPath(paths.initialShoot, true);
        state = State.TAG_READING_AND_DRIVE;
    }

    @Override
    public void loop() {
        mechanismThread.setBallPositions(sensorState.getAllPositions());
        mechanismThread.enqueueCommand(
                new MechanismThread.Command(MechanismThread.Command.Type.SHOW_LIGHTS));

        switch (state) {

            case TAG_READING_AND_DRIVE:
                updateShooterFromTag();
                if (!follower.isBusy()) {
                    shootOrder = sensorState.hasDetectedShootOrder()
                            ? sensorState.getDetectedShootOrder()
                            : DEFAULT_SHOOT_ORDER.clone();
                    follower.startTeleOpDrive();
                    resetPID();
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
                    mechanismThread.enqueueCommand(
                            new MechanismThread.Command(
                                    MechanismThread.Command.Type.SET_AUTO_INDEX, false));
                    mechanismThread.enqueueCommand(
                            new MechanismThread.Command(
                                    MechanismThread.Command.Type.SHOOT_SEQUENCE, shootOrder.clone()));
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
                    follower.setTeleOpDrive(0, 0, 0, false);
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.park, true);
                    state = State.PARKING;
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

        panelsTelemetry.debug("State", state.name());
        panelsTelemetry.debug("Shoot Order",
                shootOrder != null
                        ? shortColor(shootOrder[0]) + " " + shortColor(shootOrder[1]) + " " + shortColor(shootOrder[2])
                        : "not locked in");
        panelsTelemetry.debug("Tag Range",
                sensorState.isBasketTagVisible()
                        ? String.format("%.1f in", sensorState.getTagRange()) : "n/a");
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

    private String shortColor(ShootSequence.BallColor c) {
        if (c == ShootSequence.BallColor.GREEN)  return "G";
        if (c == ShootSequence.BallColor.PURPLE) return "P";
        return "-";
    }
}
