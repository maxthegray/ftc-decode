package org.firstinspires.ftc.teamcode.threaded;


import com.pedropathing.geometry.Pose;

public class BotState {

    private volatile boolean killThreads = false;

    // Drive state
    private volatile double driveForward = 0;
    private volatile double driveStrafe = 0;
    private volatile double driveRotate = 0;
    private volatile boolean followingPath = false;
    private volatile Pose currentPose = new Pose(0, 0, 0);

    // Ball colors
    public enum BallColor { GREEN, PURPLE, EMPTY, UNKNOWN }

    // Position indices
    public static final int POS_INTAKE = 0;
    public static final int POS_BACK_LEFT = 1;
    public static final int POS_BACK_RIGHT = 2;

    // Detected ball at each position
    private volatile BallColor[] positions = { BallColor.EMPTY, BallColor.EMPTY, BallColor.EMPTY };

    // Raw sensor values for telemetry/calibration (A and B sensor per position)
    private volatile int[] alphaA = { 0, 0, 0 };
    private volatile int[] alphaB = { 0, 0, 0 };
    private volatile int[] blueA = { 0, 0, 0 };
    private volatile int[] blueB = { 0, 0, 0 };
    private volatile int[] greenA = { 0, 0, 0 };
    private volatile int[] greenB = { 0, 0, 0 };

    // Per-position alpha thresholds
    public static int THRESHOLD_INTAKE = 200;
    public static int THRESHOLD_BACK_LEFT = 200;
    public static int THRESHOLD_BACK_RIGHT = 200;

    // Carousel state
    public static final int TICKS_PER_ROTATION = 2230;
    public static final int TICKS_PER_SLOT = TICKS_PER_ROTATION / 3;

    private volatile int carouselTargetTicks = 0;
    private volatile int carouselCurrentTicks = 0;
    private volatile boolean carouselSettled = true;

    // Carousel commands
    public enum CarouselCommand { NONE, ROTATE_EMPTY_TO_INTAKE, ROTATE_TO_KICK_GREEN, ROTATE_TO_KICK_PURPLE, ROTATE_LEFT, ROTATE_RIGHT }
    private volatile CarouselCommand carouselCommand = CarouselCommand.NONE;

    // Kicker state
    private volatile boolean kickerUp = false;
    private volatile boolean kickRequested = false;

    // Intake state
    private volatile boolean intakeForward = false;
    private volatile boolean intakeReverse = false;

    // Lights state
    private volatile boolean showLightsRequested = false;

    // Auto-index enable
    private volatile boolean autoIndexEnabled = true;

    // Shooter state
    private volatile double shooterTargetVelocity = 0;  // degrees per second
    private volatile double shooterCurrentVelocity = 0;

    // AprilTag state
    private volatile boolean basketTagVisible = false;
    private volatile double tagBearing = 0;
    private volatile double tagRange = 0;
    private volatile int tagId = -1;

    // Calculated robot pose from AprilTag
    private volatile Pose tagCalculatedPose = null;
    private volatile boolean poseUpdateRequested = false;

    // Debug info
    private volatile int detectionCount = 0;
    private volatile String cameraState = "UNKNOWN";

    // Auto-align
    private volatile boolean autoAlignEnabled = false;

    // Constructor
    public BotState() {
    }

    // Kill threads
    public void endThreads() {
        killThreads = true;
    }

    public boolean shouldKillThreads() {
        return killThreads;
    }

    // Drive getters/setters
    public void setDriveInput(double forward, double strafe, double rotate) {
        this.driveForward = forward;
        this.driveStrafe = strafe;
        this.driveRotate = rotate;
    }

    public double getDriveForward() {
        return driveForward;
    }

    public double getDriveStrafe() {
        return driveStrafe;
    }

    public double getDriveRotate() {
        return driveRotate;
    }

    public void setFollowingPath(boolean following) {
        this.followingPath = following;
    }

    public boolean isFollowingPath() {
        return followingPath;
    }

    public void setCurrentPose(Pose pose) {
        this.currentPose = pose;
    }

    public Pose getCurrentPose() {
        return currentPose;
    }

    // Color sensor setters (called by I2C threads)
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

    public void setPositionColor(int position, BallColor color) {
        if (position >= 0 && position < 3) {
            positions[position] = color;
        }
    }

    // Color sensor getters
    public BallColor getPositionColor(int position) {
        if (position >= 0 && position < 3) {
            return positions[position];
        }
        return BallColor.UNKNOWN;
    }

    public BallColor[] getAllPositions() {
        return positions.clone();
    }

    public int getAlphaA(int position) { return alphaA[position]; }
    public int getAlphaB(int position) { return alphaB[position]; }
    public int getBlueA(int position) { return blueA[position]; }
    public int getBlueB(int position) { return blueB[position]; }
    public int getGreenA(int position) { return greenA[position]; }
    public int getGreenB(int position) { return greenB[position]; }

    public int getThreshold(int position) {
        switch (position) {
            case POS_INTAKE: return THRESHOLD_INTAKE;
            case POS_BACK_LEFT: return THRESHOLD_BACK_LEFT;
            case POS_BACK_RIGHT: return THRESHOLD_BACK_RIGHT;
            default: return 200;
        }
    }

    // Ball count helpers
    public int getBallCount() {
        int count = 0;
        for (BallColor c : positions) {
            if (c == BallColor.GREEN || c == BallColor.PURPLE) count++;
        }
        return count;
    }

    public boolean isFull() {
        return getBallCount() >= 3;
    }

    public boolean isEmpty() {
        return getBallCount() == 0;
    }

    public boolean hasColor(BallColor color) {
        for (BallColor c : positions) {
            if (c == color) return true;
        }
        return false;
    }

    public int findPositionWithColor(BallColor color) {
        for (int i = 0; i < 3; i++) {
            if (positions[i] == color) return i;
        }
        return -1;
    }

    // Carousel getters/setters
    public void setCarouselCommand(CarouselCommand cmd) {
        this.carouselCommand = cmd;
    }

    public CarouselCommand getCarouselCommand() {
        return carouselCommand;
    }

    public void clearCarouselCommand() {
        this.carouselCommand = CarouselCommand.NONE;
    }

    public void setCarouselTargetTicks(int ticks) {
        this.carouselTargetTicks = ticks;
    }

    public int getCarouselTargetTicks() {
        return carouselTargetTicks;
    }

    public void setCarouselCurrentTicks(int ticks) {
        this.carouselCurrentTicks = ticks;
    }

    public int getCarouselCurrentTicks() {
        return carouselCurrentTicks;
    }

    public void setCarouselSettled(boolean settled) {
        this.carouselSettled = settled;
    }

    public boolean isCarouselSettled() {
        return carouselSettled;
    }

    // Kicker
    public void requestKick() {
        this.kickRequested = true;
    }

    public boolean isKickRequested() {
        return kickRequested;
    }

    public void clearKickRequest() {
        this.kickRequested = false;
    }

    public void setKickerUp(boolean up) {
        this.kickerUp = up;
    }

    public boolean isKickerUp() {
        return kickerUp;
    }

    // Intake
    public void setIntakeForward(boolean forward) {
        this.intakeForward = forward;
    }

    public void setIntakeReverse(boolean reverse) {
        this.intakeReverse = reverse;
    }

    public boolean isIntakeForward() {
        return intakeForward;
    }

    public boolean isIntakeReverse() {
        return intakeReverse;
    }

    public boolean isIntakeRunning() {
        return intakeForward || intakeReverse;
    }

    // Lights
    public void requestShowLights() {
        this.showLightsRequested = true;
    }

    public boolean isShowLightsRequested() {
        return showLightsRequested;
    }

    public void clearShowLightsRequest() {
        this.showLightsRequested = false;
    }

    // Auto-index
    public void setAutoIndexEnabled(boolean enabled) {
        this.autoIndexEnabled = enabled;
    }

    public boolean isAutoIndexEnabled() {
        return autoIndexEnabled;
    }

    // Shooter
    public void setShooterTargetVelocity(double velocity) {
        this.shooterTargetVelocity = velocity;
    }

    public double getShooterTargetVelocity() {
        return shooterTargetVelocity;
    }

    public void adjustShooterVelocity(double delta) {
        this.shooterTargetVelocity = Math.max(0, shooterTargetVelocity + delta);
    }

    public void setShooterCurrentVelocity(double velocity) {
        this.shooterCurrentVelocity = velocity;
    }

    public double getShooterCurrentVelocity() {
        return shooterCurrentVelocity;
    }

    // Shooter can run when intake is NOT running
    public boolean canShooterRun() {
        return !isIntakeRunning();
    }

    // AprilTag
    public void setTagData(int id, double bearing, double range) {
        this.tagId = id;
        this.tagBearing = bearing;
        this.tagRange = range;
        this.basketTagVisible = true;
    }

    public void clearTagData() {
        this.basketTagVisible = false;
        this.tagId = -1;
        this.tagBearing = 0;
        this.tagRange = 0;
    }

    public boolean isBasketTagVisible() {
        return basketTagVisible;
    }

    public double getTagBearing() {
        return tagBearing;
    }

    public double getTagRange() {
        return tagRange;
    }

    public int getTagId() {
        return tagId;
    }

    // Pose from AprilTag
    public void setTagCalculatedPose(Pose pose) {
        this.tagCalculatedPose = pose;
    }

    public Pose getTagCalculatedPose() {
        return tagCalculatedPose;
    }

    public void requestPoseUpdate() {
        this.poseUpdateRequested = true;
    }

    public boolean isPoseUpdateRequested() {
        return poseUpdateRequested;
    }

    public void clearPoseUpdateRequest() {
        this.poseUpdateRequested = false;
    }

    // Debug info
    public void setDetectionCount(int count) {
        this.detectionCount = count;
    }

    public int getDetectionCount() {
        return detectionCount;
    }

    public void setCameraState(String state) {
        this.cameraState = state;
    }

    public String getCameraState() {
        return cameraState;
    }

    // Auto-align
    public void setAutoAlignEnabled(boolean enabled) {
        this.autoAlignEnabled = enabled;
    }

    public boolean isAutoAlignEnabled() {
        return autoAlignEnabled;
    }

    public void toggleAutoAlign() {
        this.autoAlignEnabled = !autoAlignEnabled;
    }
}