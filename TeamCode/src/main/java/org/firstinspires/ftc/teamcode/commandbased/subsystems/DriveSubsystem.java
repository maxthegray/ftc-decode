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
    private boolean poseInitialized = false; //iniital pose from apriltag?

    public DriveSubsystem(HardwareMap hardwareMap) {

        follower = Constants.createFollower(hardwareMap);

        // Start with arbitrary pose - will be corrected by AprilTag
        follower.setStartingPose(new Pose(0, 0, 0));
        poseInitialized = false;
        follower.startTeleOpDrive();
    }

    @Override
    public void periodic() {
        // Update follower every cycle
//        follower.update();
    }

    /**
     * Drive using teleop controls (Pedro Pathing's built-in method).
     * This is the standard driving mode.
     *
     * @param forwardSpeed Forward/backward speed (-1 to 1)
     * @param strafeSpeed Left/right strafe speed (-1 to 1)
     * @param rotationSpeed Rotation speed (-1 to 1)
     */
    public void drive(double forwardSpeed, double strafeSpeed, double rotationSpeed) {
        follower.setTeleOpDrive(0, 0, 0);
        isFollowingPath = false;
    }

    public void stop() {
        follower.setTeleOpDrive(0, 0, 0);
        isFollowingPath = false;
    }

    /**
     * Go to origin (0, 0) with specified heading.
     * Uses Pedro Pathing's path following.
     *
     * @param targetHeading Desired heading at origin in radians
     */
    public void goToOrigin(double targetHeading) {
        Pose currentPose = follower.getPose();
        Pose originPose = new Pose(0, 0, targetHeading);

        goToPoint(originPose);
    }

    /**
     * Go to origin (0, 0) maintaining current heading.
     */
    public void goToOrigin() {
        goToOrigin(follower.getPose().getHeading());
    }

    /**
     * Go to a specific point on the field using Pedro Pathing.
     * Creates and follows a path from current position to target.
     *
     * @param targetPose Target position (X, Y, Heading in radians)
     */
    public void goToPoint(Pose targetPose) {
        Pose currentPose = follower.getPose();

        // Build path using Pedro Pathing's path builder
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, targetPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                .build();

        // Follow the path
        follower.followPath(path);
        isFollowingPath = true;
    }

    /**
     * Go to a specific point with X, Y coordinates and heading.
     *
     * @param x Target X coordinate in inches
     * @param y Target Y coordinate in inches
     * @param heading Target heading in radians
     */
    public void goToPoint(double x, double y, double heading) {
        goToPoint(new Pose(x, y, heading));
    }

    /**
     * Go to a specific point maintaining current heading.
     *
     * @param x Target X coordinate in inches
     * @param y Target Y coordinate in inches
     */
    public void goToPoint(double x, double y) {
        goToPoint(x, y, follower.getPose().getHeading());
    }

    /**
     * Check if robot is currently following an autonomous path.
     *
     * @return true if following a path, false if in teleop mode
     */
    public boolean isFollowingPath() {
        return isFollowingPath && follower.isBusy();
    }

    /**
     * Stop following the current path and return to teleop control.
     */
    public void cancelPath() {
        follower.breakFollowing();
        isFollowingPath = false;
    }

    public Follower getFollower() {
        return follower;
    }

    /**
     * Update the robot's pose using AprilTag vision correction.
     * On first detection, this initializes the robot's position.
     * After that, it provides periodic corrections.
     *
     * @param visionPose The pose calculated from AprilTag detection
     */
    public void updatePoseFromVision(Pose visionPose) {
        if (visionPose != null) {
            // Trust AprilTag 100% when it's reliable enough to call this
            follower.setPose(visionPose);

            if (!poseInitialized) {
                poseInitialized = true;
            }
        }
    }

    /**
     * Check if the robot's pose has been initialized from AprilTag.
     * Returns false until the first AprilTag detection.
     *
     * @return true if pose is initialized, false if still unknown
     */
    public boolean isPoseInitialized() {
        return poseInitialized;
    }

    /**
     * Manually mark pose as initialized (for testing or manual initialization).
     */
    public void setPoseInitialized(boolean initialized) {
        poseInitialized = initialized;
    }

    /**
     * Get the current estimated pose from odometry.
     *
     * @return Current robot pose
     */
    public Pose getPose() {
        return follower.getPose();
    }
}