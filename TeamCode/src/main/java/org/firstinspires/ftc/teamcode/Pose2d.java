package org.firstinspires.ftc.teamcode;

public class Pose2d {
    public double x;
    public double y;
    public double heading;

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    @Override
    public String toString() {
        return String.format("(%.2f, %.2f, %.2f°)", x, y, Math.toDegrees(heading));
    }
}
