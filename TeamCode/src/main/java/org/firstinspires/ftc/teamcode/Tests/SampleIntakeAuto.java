package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.threaded.Old.ControlHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.Old.ExpansionHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.Old.MechanismThread;
import org.firstinspires.ftc.teamcode.threaded.Old.SensorState;
import org.firstinspires.ftc.teamcode.threaded.Old.ShootSequence;

/**
 * ═══════════════════════════════════════════════════════════════════════════
 *  SAMPLE INTAKE AUTO — straight line, ramp-sensor-driven ball collection
 * ───────────────────────────────────────────────────────────────────────────
 *  Drives forward along one straight path with intake on.
 *  Uses the ramp sensor to gate forward motion:
 *
 *    COLLECTING  — driving forward, intake on, watching ramp sensor.
 *                  If ramp fires while carousel is busy → pause drive,
 *                  stop intake (ball parks on ramp), go to HOLDING.
 *                  If ramp fires while carousel is idle → do nothing,
 *                  ball flows straight through uninterrupted.
 *
 *    HOLDING     — drive paused, intake stopped, ball held on ramp.
 *                  Once carousel finishes indexing AND carousel is not full:
 *                  resume drive, intake back on, return to COLLECTING.
 *
 *  Finishes when the path ends OR 3 balls are collected, whichever first.
 * ═══════════════════════════════════════════════════════════════════════════
 */
@Disabled
@Autonomous(name = "Sample Intake Auto", group = "Auto")
public class SampleIntakeAuto extends OpMode {

    // ════════════════════════════════════════════════════════════════════════
    //  POSES  — edit these to match your field setup
    // ════════════════════════════════════════════════════════════════════════

    private final Pose startPose = new Pose(25.5, 63, Math.toRadians(0));
    private final Pose endPose   = new Pose(25.5, 20, Math.toRadians(0));

    // ════════════════════════════════════════════════════════════════════════
    //  STATE MACHINE
    // ════════════════════════════════════════════════════════════════════════

    private enum State {
        /** Driving forward, intake on, ramp sensor watched every frame. */
        COLLECTING,
        /**
         * Ramp fired while carousel was busy.
         * Drive paused, intake stopped, ball held on ramp by still wheel.
         * Resumes once carousel is idle and there is room for the ball.
         */
        HOLDING,
        /** All done — 3 balls collected or path finished. */
        DONE
    }

    private State state = State.COLLECTING;

    // ════════════════════════════════════════════════════════════════════════
    //  HARDWARE / THREADS
    // ════════════════════════════════════════════════════════════════════════

    private Follower              follower;
    private PathChain             collectPath;
    private MechanismThread       mechanismThread;
    private ControlHubI2CThread   controlHubI2C;
    private ExpansionHubI2CThread expansionHubI2C;
    private SensorState           sensorState;

    // ════════════════════════════════════════════════════════════════════════
    //  LIFECYCLE
    // ════════════════════════════════════════════════════════════════════════

    @Override
    public void init() {
        sensorState = new SensorState(SensorState.Alliance.BLUE);

        mechanismThread = new MechanismThread(hardwareMap);
        mechanismThread.setSensorState(sensorState);

        controlHubI2C   = new ControlHubI2CThread(sensorState, hardwareMap);
        expansionHubI2C = new ExpansionHubI2CThread(sensorState, hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.5);
        collectPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setTangentHeadingInterpolation()
                .build();

        // Auto-index on so carousel rotates when a ball enters
        mechanismThread.enqueueCommand(
                new MechanismThread.Command(MechanismThread.Command.Type.SET_AUTO_INDEX, true));

        telemetry.addData("Status", "Initialized — ready to start");
        telemetry.update();
    }

    @Override
    public void start() {
        mechanismThread.start();
        controlHubI2C.start();
        expansionHubI2C.start();

        mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);
        follower.followPath(collectPath, true);
        state = State.COLLECTING;
    }

    @Override
    public void loop() {
        mechanismThread.setBallPositions(sensorState.getAllPositions());

        boolean rampTriggered = sensorState.isRampTriggered();
        boolean mechIdle      = mechanismThread.isIdle();
        boolean full          = countBalls() >= 3;

        switch (state) {

            // ── Driving forward, intake on ───────────────────────────────────
            case COLLECTING:

                if (full) {
                    // Carousel full — stop everything
                    finish();
                    break;
                }

                mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);

                if (rampTriggered && !mechIdle) {
                    // New ball on ramp, carousel still busy — park ball, pause drive
                    mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
                    follower.breakFollowing();
                    state = State.HOLDING;
                    break;
                }

                if (!follower.isBusy()) {
                    // Reached end of path — whatever we have is what we got
                    finish();
                }
                break;

            // ── Drive paused, ball held on ramp ──────────────────────────────
            case HOLDING:
                mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);

                if (full) {
                    finish();
                    break;
                }

                if (mechIdle) {
                    // Carousel done — resume drive and intake
                    follower.followPath(collectPath, true);
                    mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);
                    state = State.COLLECTING;
                }
                break;

            case DONE:
                break;
        }

        follower.update();

        // ── Telemetry ─────────────────────────────────────────────────────
        telemetry.addData("State",     state.name());
        telemetry.addData("Balls",     "%d / 3", countBalls());
        telemetry.addData("Ramp",      sensorState.isRampTriggered() ? "TRIGGERED" : "clear");
        telemetry.addData("Mechanism", mechIdle ? "IDLE" : mechanismThread.getStateDebug());
        telemetry.addData("Driving",   follower.isBusy() ? "YES" : "NO");
        telemetry.update();
    }

    @Override
    public void stop() {
        mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
        mechanismThread.kill();
        sensorState.kill();
        try {
            mechanismThread.join(300);
            controlHubI2C.join(300);
            expansionHubI2C.join(300);
        } catch (InterruptedException ignored) {}
    }

    // ════════════════════════════════════════════════════════════════════════
    //  HELPERS
    // ════════════════════════════════════════════════════════════════════════

    private void finish() {
        mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
        follower.breakFollowing();
        state = State.DONE;
    }

    private int countBalls() {
        int count = 0;
        for (ShootSequence.BallColor c : sensorState.getAllPositions()) {
            if (c == ShootSequence.BallColor.GREEN || c == ShootSequence.BallColor.PURPLE) count++;
        }
        return count;
    }
}