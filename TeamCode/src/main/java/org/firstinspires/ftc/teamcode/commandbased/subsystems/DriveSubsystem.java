package org.firstinspires.ftc.teamcode.commandbased.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DriveSubsystem extends SubsystemBase {

    private final Follower follower;
    private boolean isFollowingPath = false;
    private boolean poseInitialized = false;

    public DriveSubsystem(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        poseInitialized = false;
        follower.startTeleOpDrive();
    }

    @Override
    public void periodic() {
        follower.update();
    }

    public void drive(double forwardSpeed, double strafeSpeed, double rotationSpeed) {
        follower.setTeleOpDrive(forwardSpeed, strafeSpeed, rotationSpeed);
        isFollowingPath = false;
    }

    public void initializeTeleOpDrive() {
        follower.startTeleOpDrive(true);
    }

    public void stop() {
//        follower.setTeleOpDrive(0, 0, 0);
        isFollowingPath = false;
    }

    public void goToOrigin(double targetHeading) {
        Pose currentPose = follower.getPose();
        Pose originPose = new Pose(0, 0, targetHeading);
        goToPoint(originPose);
    }

    public void goToOrigin() {
        goToOrigin(follower.getPose().getHeading());
    }

    public void goToPoint(Pose targetPose) {
        Pose currentPose = follower.getPose();

        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, targetPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                .build();

        follower.followPath(path);
        follower.update();
        isFollowingPath = true;
    }

    public void goToPoint(double x, double y, double heading) {
        goToPoint(new Pose(x, y, heading));
    }

    public void goToPoint(double x, double y) {
        goToPoint(x, y, follower.getPose().getHeading());
    }

    public boolean isFollowingPath() {
        return isFollowingPath && follower.isBusy();
    }

    public void cancelPath() {
        follower.breakFollowing();
        isFollowingPath = false;
    }

    public Follower getFollower() {
        return follower;
    }

    public void updatePoseFromVision(Pose visionPose) {
        if (visionPose != null) {
            follower.setPose(visionPose);
            if (!poseInitialized) {
                poseInitialized = true;
            }
        }
    }

    public boolean isPoseInitialized() {
        return poseInitialized;
    }

    public void setPoseInitialized(boolean initialized) {
        poseInitialized = initialized;
    }

    public Pose getPose() {
        return follower.getPose();
    }
}