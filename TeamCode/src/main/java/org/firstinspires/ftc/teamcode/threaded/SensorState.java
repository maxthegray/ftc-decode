package org.firstinspires.ftc.teamcode.threaded;

import com.pedropathing.geometry.Pose;

/**
 * Shared state for sensor data, shooter, drive, and AprilTag.
 *
 * This is a slimmed-down version of the old BotState.
 * MechanismThread handles its own internal state separately.
 */
public class SensorState {

    // ==================== ALLIANCE ====================
    public enum Alliance { RED, BLUE }
    private final Alliance alliance;

    public SensorState(Alliance alliance) {
        this.alliance = alliance;
    }

    public Alliance getAlliance() { return alliance; }

    // ==================== THREAD CONTROL ====================
    private volatile boolean killThreads = false;

    public void kill() { killThreads = true; }
    public boolean shouldKillThreads() { return killThreads; }

    // ==================== UPDATE RATES ====================
    public static final long DRIVE_UPDATE_MS = 1;
    public static final long SHOOTER_UPDATE_MS = 50;
    public static final long CAMERA_UPDATE_MS = 30;
    public static final long I2C_UPDATE_MS = 50;

    // ==================== BALL POSITIONS ====================
    public static final int POS_INTAKE = 0;
    public static final int POS_BACK_LEFT = 1;
    public static final int POS_BACK_RIGHT = 2;

    // When To Read Flags
    private volatile boolean carouselSpinning = false;

    public void setCarouselSpinning(boolean spinning) { this.carouselSpinning = spinning; }
    public boolean isCarouselSpinning() { return carouselSpinning; }

    private volatile   ShootSequence.BallColor[] positions = {
            ShootSequence.BallColor.EMPTY,
            ShootSequence.BallColor.EMPTY,
            ShootSequence.BallColor.EMPTY
    };

    public void setPositionColor(int position,   ShootSequence.BallColor color) {
        if (position >= 0 && position < 3) {
            positions[position] = color;
        }
    }

    public   ShootSequence.BallColor getPositionColor(int position) {
        if (position >= 0 && position < 3) {
            return positions[position];
        }
        return   ShootSequence.BallColor.UNKNOWN;
    }

    public   ShootSequence.BallColor[] getAllPositions() {
        return positions.clone();
    }

    // ==================== RAW SENSOR VALUES ====================
    private volatile double[] distanceA = { 0, 0, 0 };  // mm
    private volatile double[] distanceB = { 0, 0, 0 };  // mm
    private volatile int[] blueA = { 0, 0, 0 };
    private volatile int[] blueB = { 0, 0, 0 };
    private volatile int[] greenA = { 0, 0, 0 };
    private volatile int[] greenB = { 0, 0, 0 };

    // Distance thresholds (mm) — ball is present when distance < threshold
    // TODO: Tune these values on the actual robot
    public static double THRESHOLD_INTAKE_A = 75.0; //120 idle 33
    public static double THRESHOLD_INTAKE_B = 75.0; //93 idle  33
    public static double THRESHOLD_BACK_LEFT_A = 40;
    public static double THRESHOLD_BACK_LEFT_B = 70;
    public static double THRESHOLD_BACK_RIGHT_A = 120; //87 idle 17 with ball
    public static double THRESHOLD_BACK_RIGHT_B = 120; //147 idle 12 with ball

    public void setSensorValuesA(int position, double distance, int blue, int green) {
        if (position >= 0 && position < 3) {
            distanceA[position] = distance;
            blueA[position] = blue;
            greenA[position] = green;
        }
    }

    public void setSensorValuesB(int position, double distance, int blue, int green) {
        if (position >= 0 && position < 3) {
            distanceB[position] = distance;
            blueB[position] = blue;
            greenB[position] = green;
        }
    }

    public double getDistanceA(int pos) { return distanceA[pos]; }
    public double getDistanceB(int pos) { return distanceB[pos]; }
    public int getBlueA(int pos) { return blueA[pos]; }
    public int getBlueB(int pos) { return blueB[pos]; }
    public int getGreenA(int pos) { return greenA[pos]; }
    public int getGreenB(int pos) { return greenB[pos]; }

    public double getThresholdA(int position) {
        switch (position) {
            case POS_INTAKE: return THRESHOLD_INTAKE_A;
            case POS_BACK_LEFT: return THRESHOLD_BACK_LEFT_A;
            case POS_BACK_RIGHT: return THRESHOLD_BACK_RIGHT_A;
            default: return 50.0;
        }
    }

    public double getThresholdB(int position) {
        switch (position) {
            case POS_INTAKE: return THRESHOLD_INTAKE_B;
            case POS_BACK_LEFT: return THRESHOLD_BACK_LEFT_B;
            case POS_BACK_RIGHT: return THRESHOLD_BACK_RIGHT_B;
            default: return 50.0;
        }
    }

    // ==================== SHOOTER ====================
    private volatile double shooterTargetVelocity = 0;
    private volatile double shooterCurrentVelocity = 0;
    public static double VELOCITY_TOLERANCE = 10;

    // Distance-to-velocity polynomial
    private static final double COEFF_A = 0.000000780128;
    private static final double COEFF_B = -0.0001727;
    private static final double COEFF_C = 0.0168311;
    private static final double COEFF_D = 0.190235;
    private static final double COEFF_E = 128.62877;

    public void setShooterTargetVelocity(double v) { this.shooterTargetVelocity = v; }
    public double getShooterTargetVelocity() { return shooterTargetVelocity; }

    public void setShooterCurrentVelocity(double v) { this.shooterCurrentVelocity = v; }
    public double getShooterCurrentVelocity() { return shooterCurrentVelocity; }

    public void setVelocityFromDistance(double distance) {
        double d = distance;
        shooterTargetVelocity = COEFF_A*d*d*d*d + COEFF_B*d*d*d + COEFF_C*d*d + COEFF_D*d + COEFF_E;
        shooterTargetVelocity = Math.max(0, shooterTargetVelocity);
    }

    public boolean isShooterReady() {
        return shooterTargetVelocity > 0 &&
                Math.abs(shooterCurrentVelocity - shooterTargetVelocity) <= VELOCITY_TOLERANCE;
    }

    // ==================== DRIVE ====================
    private volatile double driveForward = 0;
    private volatile double driveStrafe = 0;
    private volatile double driveRotate = 0;
    private volatile Pose currentPose = new Pose(0, 0, 0);
    private volatile boolean followingPath = false;

    public void setDriveInput(double forward, double strafe, double rotate) {
        this.driveForward = forward;
        this.driveStrafe = strafe;
        this.driveRotate = rotate;
    }

    public double getDriveForward() { return driveForward; }
    public double getDriveStrafe() { return driveStrafe; }
    public double getDriveRotate() { return driveRotate; }

    public void setCurrentPose(Pose pose) { this.currentPose = pose; }
    public Pose getCurrentPose() { return currentPose; }

    public void setFollowingPath(boolean following) { this.followingPath = following; }
    public boolean isFollowingPath() { return followingPath; }

    // ==================== APRILTAG ====================
    private volatile boolean basketTagVisible = false;
    private volatile double tagBearing = 0;
    private volatile double tagRange = 0;
    private volatile double tagYaw = 0;
    private volatile int tagId = -1;
    private volatile Pose tagCalculatedPose = null;
    private volatile boolean poseUpdateRequested = false;

    public static final double TARGET_OFFSET_INCHES = 45.5 / 2.54;

    // ── Bearing interpolation ────────────────────────────────────────────────
    // Snapshot the robot heading (radians) at the moment each camera frame
    // arrives. Between frames, DriveThread can predict the current bearing by
    // compensating for heading change since the snapshot.
    private volatile double headingAtLastFrame = 0;        // radians
    private volatile double targetBearingAtLastFrame = 0;  // degrees (computed)

    public void setTagData(int id, double bearing, double range, double yaw) {
        this.tagId = id;
        this.tagBearing = bearing;
        this.tagRange = range;
        this.tagYaw = yaw;
        this.basketTagVisible = true;

        // Snapshot heading and pre-compute target bearing at this frame
        this.headingAtLastFrame = currentPose.getHeading();
        this.targetBearingAtLastFrame = computeTargetBearing(bearing, range, yaw);
    }

    public void clearTagData() {
        this.basketTagVisible = false;
        this.tagId = -1;
    }

    public boolean isBasketTagVisible() { return basketTagVisible; }
    public double getTagBearing() { return tagBearing; }
    public double getTagRange() { return tagRange; }
    public double getTagYaw() { return tagYaw; }

    public double getTargetBearing() {
        if (!basketTagVisible) return 0;
        return computeTargetBearing(tagBearing, tagRange, tagYaw);
    }

    /**
     * Predict the current target bearing by compensating for heading change
     * since the last camera frame.
     *
     * Between frames the robot may have rotated, which shifts the tag's
     * apparent bearing by the opposite amount. The gyro/odometry heading
     * is accurate over 30ms windows, so this prediction is essentially exact.
     *
     * @param currentHeadingRad current robot heading in radians (from Pose)
     * @return predicted target bearing in degrees
     */
    public double getInterpolatedBearing(double currentHeadingRad) {
        if (!basketTagVisible) return 0;
        double headingDeltaDeg = Math.toDegrees(currentHeadingRad - headingAtLastFrame);
        return targetBearingAtLastFrame - headingDeltaDeg;
    }

    /** Raw (non-interpolated) target bearing from last camera frame, for telemetry comparison. */
    public double getRawTargetBearing() {
        return targetBearingAtLastFrame;
    }

    public double getHeadingAtLastFrame() {
        return headingAtLastFrame;
    }

    /**
     * Compute the adjusted target bearing accounting for the offset between
     * the tag center and the actual target point.
     */
    private double computeTargetBearing(double bearing, double range, double yaw) {
        double r = range;
        double d = TARGET_OFFSET_INCHES;
        double yawRad = Math.toRadians(yaw);

        double rtSquared = r*r + d*d + 2*r*d*Math.cos(yawRad);
        double rt = Math.sqrt(rtSquared);

        double sinBeta = d * Math.sin(yawRad) / rt;
        sinBeta = Math.max(-1.0, Math.min(1.0, sinBeta));
        double beta = Math.toDegrees(Math.asin(sinBeta));

        return bearing + beta;
    }

    public void setTagCalculatedPose(Pose pose) { this.tagCalculatedPose = pose; }
    public Pose getTagCalculatedPose() { return tagCalculatedPose; }

    public void requestPoseUpdate() { this.poseUpdateRequested = true; }
    public boolean isPoseUpdateRequested() { return poseUpdateRequested; }
    public void clearPoseUpdateRequest() { this.poseUpdateRequested = false; }

    // Auto-align
    private volatile boolean autoAlignEnabled = false;
    public static double ALIGN_P = 0.006;
    public static double ALIGN_I = 0.0005;
    public static double ALIGN_D = 0.0002;
    public static double ALIGN_DEADBAND = 1.0;
    public static double ALIGN_D_ALPHA = 0.19;  // D low-pass filter: 0=raw, 1=max smoothing

    // ==================== BASKET POSITIONS (for odometry fallback) ====================
    // These match CameraThread's tag positions.
    // Blue basket (tag 20):
    private static final double BLUE_BASKET_X = 58.35 + 72;   // 130.35
    private static final double BLUE_BASKET_Y = 55.63 - 72;   // -16.37

    // Red basket (tag 24):
    private static final double RED_BASKET_X = -58.35 - 72;   // -130.35
    private static final double RED_BASKET_Y = 55.63 - 72;    // -16.37

    /**
     * Estimate distance to basket using odometry pose.
     * Less accurate than AprilTag range but always available.
     */
    public double getOdometryDistanceToBasket() {
        Pose pose = currentPose;
        double bx = (alliance == Alliance.BLUE) ? BLUE_BASKET_X : RED_BASKET_X;
        double by = (alliance == Alliance.BLUE) ? BLUE_BASKET_Y : RED_BASKET_Y;
        double dx = pose.getX() - bx;
        double dy = pose.getY() - by;
        return Math.sqrt(dx * dx + dy * dy);
    }

    public void setAutoAlignEnabled(boolean enabled) { this.autoAlignEnabled = enabled; }
    public boolean isAutoAlignEnabled() { return autoAlignEnabled; }
    public void toggleAutoAlign() { this.autoAlignEnabled = !autoAlignEnabled; }

    // ==================== RAMP SENSOR ====================
    // True when a ball is detected on the intake ramp (analog sensor "curvy")
    private volatile boolean rampSensorTriggered = false;

    public void setRampTriggered(boolean triggered) { this.rampSensorTriggered = triggered; }
    public boolean isRampTriggered() { return rampSensorTriggered; }

    // Voltage threshold — matches TouchSensorTest
    public static final double RAMP_SENSOR_THRESHOLD = 0.200;

    // Shoot order from AprilTag
    private volatile   ShootSequence.BallColor[] detectedShootOrder = null;

    public void setDetectedShootOrder(  ShootSequence.BallColor[] order) {
        this.detectedShootOrder = order != null ? order.clone() : null;
    }

    public   ShootSequence.BallColor[] getDetectedShootOrder() {
        return detectedShootOrder != null ? detectedShootOrder.clone() : null;
    }

    public boolean hasDetectedShootOrder() {
        return detectedShootOrder != null;
    }
}