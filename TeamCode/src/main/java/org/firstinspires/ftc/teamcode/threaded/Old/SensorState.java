package org.firstinspires.ftc.teamcode.threaded.Old;

import com.pedropathing.geometry.Pose;

/**
 * Shared state for sensor data, shooter, drive, and AprilTag.
 *
 * This is a slimmed-down version of the old BotState.
 * MechanismThread handles its own internal state separately.
 */
public class SensorState {

    // ==================== THREAD CONTROL ====================
    private volatile boolean killThreads = false;

    public void kill() { killThreads = true; }
    public boolean shouldKillThreads() { return killThreads; }

    // ==================== UPDATE RATES ====================
    public static final long DRIVE_UPDATE_MS = 10;
    public static final long SHOOTER_UPDATE_MS = 50;
    public static final long CAMERA_UPDATE_MS = 30;
    public static final long I2C_UPDATE_MS = 50;

    // ==================== BALL POSITIONS ====================
    public static final int POS_INTAKE = 0;
    public static final int POS_BACK_LEFT = 1;
    public static final int POS_BACK_RIGHT = 2;

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
    private volatile int[] alphaA = { 0, 0, 0 };
    private volatile int[] alphaB = { 0, 0, 0 };
    private volatile int[] blueA = { 0, 0, 0 };
    private volatile int[] blueB = { 0, 0, 0 };
    private volatile int[] greenA = { 0, 0, 0 };
    private volatile int[] greenB = { 0, 0, 0 };

    // Thresholds
    public static int THRESHOLD_INTAKE_A = 80;
    public static int THRESHOLD_INTAKE_B = 125;
    public static int THRESHOLD_BACK_LEFT_A = 300;
    public static int THRESHOLD_BACK_LEFT_B = 200;
    public static int THRESHOLD_BACK_RIGHT_A = 300;
    public static int THRESHOLD_BACK_RIGHT_B = 200;

    public void setSensorValuesA(int position, int alpha, int blue, int green) {
        if (position >= 0 && position < 3) {
            alphaA[position] = alpha;
            blueA[position] = blue;
            greenA[position] = green;
        }
    }

    public void setSensorValuesB(int position, int alpha, int blue, int green) {
        if (position >= 0 && position < 3) {
            alphaB[position] = alpha;
            blueB[position] = blue;
            greenB[position] = green;
        }
    }

    public int getAlphaA(int pos) { return alphaA[pos]; }
    public int getAlphaB(int pos) { return alphaB[pos]; }
    public int getBlueA(int pos) { return blueA[pos]; }
    public int getBlueB(int pos) { return blueB[pos]; }
    public int getGreenA(int pos) { return greenA[pos]; }
    public int getGreenB(int pos) { return greenB[pos]; }

    public int getThresholdA(int position) {
        switch (position) {
            case POS_INTAKE: return THRESHOLD_INTAKE_A;
            case POS_BACK_LEFT: return THRESHOLD_BACK_LEFT_A;
            case POS_BACK_RIGHT: return THRESHOLD_BACK_RIGHT_A;
            default: return 200;
        }
    }

    public int getThresholdB(int position) {
        switch (position) {
            case POS_INTAKE: return THRESHOLD_INTAKE_B;
            case POS_BACK_LEFT: return THRESHOLD_BACK_LEFT_B;
            case POS_BACK_RIGHT: return THRESHOLD_BACK_RIGHT_B;
            default: return 200;
        }
    }

    // ==================== SHOOTER ====================
    private volatile double shooterTargetVelocity = 0;
    private volatile double shooterCurrentVelocity = 0;
    public static double VELOCITY_TOLERANCE = 20;

    // Distance-to-velocity polynomial
    private static final double COEFF_A = 0.00000043;
    private static final double COEFF_B = -0.0001927;
    private static final double COEFF_C = 0.026899;
    private static final double COEFF_D = -0.402824;
    private static final double COEFF_E = 136.48202;

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

    public void setTagData(int id, double bearing, double range, double yaw) {
        this.tagId = id;
        this.tagBearing = bearing;
        this.tagRange = range;
        this.tagYaw = yaw;
        this.basketTagVisible = true;
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

        double r = tagRange;
        double d = TARGET_OFFSET_INCHES;
        double yawRad = Math.toRadians(tagYaw);

        double rtSquared = r*r + d*d + 2*r*d*Math.cos(yawRad);
        double rt = Math.sqrt(rtSquared);

        double sinBeta = d * Math.sin(yawRad) / rt;
        sinBeta = Math.max(-1.0, Math.min(1.0, sinBeta));
        double beta = Math.toDegrees(Math.asin(sinBeta));

        return tagBearing + beta;
    }

    public void setTagCalculatedPose(Pose pose) { this.tagCalculatedPose = pose; }
    public Pose getTagCalculatedPose() { return tagCalculatedPose; }

    public void requestPoseUpdate() { this.poseUpdateRequested = true; }
    public boolean isPoseUpdateRequested() { return poseUpdateRequested; }
    public void clearPoseUpdateRequest() { this.poseUpdateRequested = false; }

    // Auto-align
    private volatile boolean autoAlignEnabled = false;
    public static double ALIGN_P = 0.007;
    public static double ALIGN_I = 0.0003;
    public static double ALIGN_D = 0.0006;
    public static double ALIGN_DEADBAND = 1.0;

    public void setAutoAlignEnabled(boolean enabled) { this.autoAlignEnabled = enabled; }
    public boolean isAutoAlignEnabled() { return autoAlignEnabled; }
    public void toggleAutoAlign() { this.autoAlignEnabled = !autoAlignEnabled; }

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