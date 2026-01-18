package org.firstinspires.ftc.teamcode.threaded;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DriveThread extends Thread {

    private final BotState state;
    private final Follower follower;

    // Auto-align constants
    private static final double ALIGN_POWER = 0.25;

    public DriveThread(BotState state, HardwareMap hardwareMap) {
        this.state = state;
        this.setPriority(Thread.MAX_PRIORITY);  // High priority for drive

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleOpDrive();

    }

    @Override
    public void run() {
        while (!state.shouldKillThreads()) {

            // Check for pose update request from AprilTag
            if (state.isPoseUpdateRequested()) {
                Pose tagPose = state.getTagCalculatedPose();
                if (tagPose != null) {
                    follower.setPose(tagPose);
                }
                state.clearPoseUpdateRequest();
            }

            // Get drive input from state
            double forward = state.getDriveForward();
            double strafe = state.getDriveStrafe();
            double rotate = state.getDriveRotate();

            // Apply auto-align if enabled and tag visible
            if (state.isAutoAlignEnabled() && state.isBasketTagVisible()) {
                double bearing = state.getTagBearing();
                // Proportional control: rotate = ALIGN_POWER * (bearing / 30)
                rotate = ALIGN_POWER * (bearing / 35.0);
            }

            // Apply to follower
            follower.setTeleOpDrive(forward, strafe, rotate, true);
            follower.update();

            // Update state with current pose
            state.setCurrentPose(follower.getPose());
            state.setFollowingPath(follower.isBusy());

            try {
                Thread.sleep(BotState.DRIVE_UPDATE_MS);
            } catch (InterruptedException e) {
                break;
            }
        }
    }

    public Follower getFollower() {
        return follower;
    }
}