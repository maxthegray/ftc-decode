package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

public class Localization {

    private Pose2d odometryPose;   // odo reading
    private Pose2d aprilTagPose;   // apriltag reading
    private Pose2d fusedPose;      // blended estimate

    private double alpha = 0.2;    // weight factor: 0.8 = trust apriltag more

    GoBildaPinpointDriver.GoBildaOdometryPods pod; //idk?

    public Localization() {
        odometryPose = new Pose2d(0, 0, 0);
        aprilTagPose = null;
        fusedPose = new Pose2d(0, 0, 0);
    }

    // update odo reading
    public void updateOdometry(Pose2d odoPose) {
        this.odometryPose = odoPose;
        fusePoses();
    }

    // update apriltag (null if not visible)
    public void updateAprilTag(Pose2d tagPose) {
        this.aprilTagPose = tagPose;
        fusePoses();
    }

    // blend the two poses (apriltag weighted more by alpha)
    private void fusePoses() {
        if (aprilTagPose != null) {
            double x = alpha * odometryPose.x + (1 - alpha) * aprilTagPose.x;
            double y = alpha * odometryPose.y + (1 - alpha) * aprilTagPose.y;

            // heading
            double heading = alpha * odometryPose.heading + (1 - alpha) * aprilTagPose.heading;

            fusedPose = new Pose2d(x, y, heading);
        } else {
            fusedPose = odometryPose;
        }
    }

    // Get the best estimate of current position
    public Pose2d getPoseEstimate() {
        return fusedPose;
    }
}
