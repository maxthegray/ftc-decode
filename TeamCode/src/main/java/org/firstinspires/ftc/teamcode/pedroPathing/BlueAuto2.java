package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
@Configurable // Panels
public class BlueAuto2 extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState; // Current autonomous path state (state machine)
    private ElapsedTime time;

    // New Set
    private final Pose startPose = new Pose(22, 123, Math.toRadians(90)); //starting pose
    private final Pose pose2 = new Pose(44, 98, Math.toRadians(145)); //first shoot
    private final Pose pose3 = new Pose(24, 84, Math.toRadians(180)); //middle balls
    private final Pose pose4 = new Pose(44, 96, Math.toRadians(140)); //second shoot
    private final Pose pose5 = new Pose(24, 36, Math.toRadians(180)); //far balls
    private final Pose pose6 = new Pose(60, 13, Math.toRadians(140)); //third shoot
    private final Pose pose21 = new Pose (74.3,77.8); //control 1
    private final Pose pose31 = new Pose(99.5,26.2); //control 2




    private Path preShoot;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2;
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        preShoot = new Path(new BezierLine(startPose, pose2));
        preShoot.setLinearHeadingInterpolation(startPose.getHeading(), pose2.getHeading());
        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(pose2, pose21, pose3))
                .setLinearHeadingInterpolation(pose2.getHeading(), pose3.getHeading())
                .build();
        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(pose3,pose31, pose4))
                .setLinearHeadingInterpolation(pose3.getHeading(), pose4.getHeading())
                .build();
        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pose4, pose5))
                .setLinearHeadingInterpolation(pose4.getHeading(), pose5.getHeading())
                .build();
        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pose5,pose6))
                .setLinearHeadingInterpolation(pose5.getHeading(), pose6.getHeading())
                .build();
    }
    @Override
    public void init() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);


        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    @Override
    public void stop() {
        // Pass final pose to TeleOp
        BlueTeleOp.startingPose = pose6;
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(preShoot);
                setPathState(1);
                break;
            case 1:
                // Wait for path to finish
                if(!follower.isBusy()) {
                    actionTimer.resetTimer();  // Start the 5-second timer
                    setPathState(2);
                }
                break;
            case 2:
                // Wait 5 seconds after preShoot completes
                if(actionTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(grabPickup1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(4);
                }
                break;
            case 4:
                // Wait 5 seconds after grabPickup1 completes
                if(actionTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(scorePickup1, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(6);
                }
                break;
            case 6:
                // Wait 5 seconds after scorePickup1 completes
                if(actionTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(grabPickup2, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    actionTimer.resetTimer();
                    setPathState(8);
                }
                break;
            case 8:
                // Wait 5 seconds after grabPickup2 completes
                if(actionTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(scorePickup2, true);
                    setPathState(-1);
                }
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
