package org.firstinspires.ftc.teamcode.threaded;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DriveThread extends Thread {

    private final BotState state;
    private final Follower follower;

    private static final long UPDATE_INTERVAL_MS = 10;

    public DriveThread(BotState state, HardwareMap hardwareMap) {
        this.state = state;
        this.follower = Constants.createFollower(hardwareMap);
    }

    public void initialize() {
        follower.startTeleOpDrive();
    }

    public Follower getFollower() {
        return follower;
    }

    @Override
    public void run() {
        while (!state.shouldKillThreads()) {

            // Get drive input from state
            double forward = state.getDriveForward();
            double strafe = state.getDriveStrafe();
            double rotate = state.getDriveRotate();

            // Apply to follower
            follower.setTeleOpDrive(forward, strafe, rotate, true);
            follower.update();

            // Update state with current pose
            state.setCurrentPose(follower.getPose());
            state.setFollowingPath(follower.isBusy());

            try {
                Thread.sleep(UPDATE_INTERVAL_MS);
            } catch (InterruptedException e) {
                break;
            }
        }
    }
}