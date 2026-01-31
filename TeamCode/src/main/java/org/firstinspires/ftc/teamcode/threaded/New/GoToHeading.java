package org.firstinspires.ftc.teamcode.threaded.New;

import static java.lang.Math.abs;
import static java.lang.Math.PI;

/**
 * Simple "go to heading" controller - rotates robot to face a target direction.
 * No translation, just rotation.
 */
public class GoToHeading {

    private double targetHeading;
    private double headingTolerance = 0.05;   // radians (~3 degrees)
    private double slowdownRange = 0.5;       // radians (~30 degrees) - start slowing down
    private double turnSpeed = 0.5;           // max turn power (0 to 1)
    private double minTurnSpeed = 0.12;       // minimum to overcome friction

    private float[] driveVector = new float[3];
    private boolean isAtTarget = false;

    public GoToHeading() {
        targetHeading = 0;
    }

    /**
     * Set target heading
     * @param heading target angle in radians
     */
    public void setTargetHeading(double heading) {
        this.targetHeading = normalizeAngle(heading);
        this.isAtTarget = false;
    }

    /**
     * Set how close the heading needs to be to consider it "at target"
     * @param tolerance angle in radians
     */
    public void setHeadingTolerance(double tolerance) {
        this.headingTolerance = tolerance;
    }

    /**
     * Set the maximum turn speed
     * @param speed value from 0 to 1
     */
    public void setTurnSpeed(double speed) {
        this.turnSpeed = speed;
    }

    /**
     * Call this every loop iteration
     * @param state the BotState containing current heading from OTOS
     * @return drive vector [magnitude, angle, turn] to pass to XdriveTrain
     */
    public float[] update(BotState state) {
        double currentHeading = state.getHeading();

        // Calculate shortest path to target (handles wrap-around)
        double error = normalizeAngle(targetHeading - currentHeading);

        // Are we there yet?
        if (abs(error) < headingTolerance) {
            isAtTarget = true;
            driveVector[0] = 0;  // no translation
            driveVector[1] = 0;
            driveVector[2] = 0;  // no turn
            return driveVector;
        }

        // Proportional turn speed - slow down as we approach
        double turn = turnSpeed;
        if (abs(error) < slowdownRange) {
            turn = turnSpeed * (abs(error) / slowdownRange);
            turn = Math.max(turn, minTurnSpeed);
        }

        // Turn the correct direction (negative error = turn negative)
        if (error < 0) {
            turn = -turn;
        }

        driveVector[0] = 0;             // no translation
        driveVector[1] = 0;
        driveVector[2] = (float) turn;  // rotation only

        return driveVector;
    }


    private double normalizeAngle(double angle) {
        while (angle > PI) angle -= 2 * PI;
        while (angle < -PI) angle += 2 * PI;
        return angle;
    }

    public boolean isAtTarget() {
        return isAtTarget;
    }

    public double getHeadingError(BotState state) {
        return normalizeAngle(targetHeading - state.getHeading());
    }

    public double getTargetHeading() {
        return targetHeading;
    }
}