package org.firstinspires.ftc.teamcode.threaded;

import org.firstinspires.ftc.teamcode.threaded.BotState.CarouselCommand;

/**
 * Simple shoot sequence - no color detection.
 * Just aligns, shoots, rotates clockwise, repeat 3 times.
 */
public class SimpleShootSequence {

    private final BotState state;

    private static final double DEFAULT_VELOCITY = 210.0;
    private static final double BEARING_TOLERANCE = 3.0;
    private static final int TOTAL_BALLS = 3;

    public enum ShootState {
        IDLE,
        ALIGN,
        WAIT_ALIGN,
        SPIN_UP,
        WAIT_SHOOTER,
        KICK,
        WAIT_KICK_DONE,
        ROTATE,
        WAIT_ROTATE
    }

    private ShootState shootState = ShootState.IDLE;
    private int ballsShot = 0;
    private long stateStartTime = 0;
    private String debugMessage = "";

    public SimpleShootSequence(BotState state) {
        this.state = state;
    }

    public void start() {
        if (isActive()) return;
        ballsShot = 0;
        changeState(ShootState.ALIGN);
        debugMessage = "Starting simple sequence";
    }

    public void cancel() {
        shootState = ShootState.IDLE;
        ballsShot = 0;
        state.setShooterTargetVelocity(0);
        state.setAutoAlignEnabled(false);
        debugMessage = "Cancelled";
    }

    public boolean isActive() {
        return shootState != ShootState.IDLE;
    }

    public ShootState getState() {
        return shootState;
    }

    public String getStateName() {
        return shootState.toString();
    }

    public int getBallsShot() {
        return ballsShot;
    }

    public String getDebugMessage() {
        return debugMessage;
    }

    public long getStateTime() {
        return System.currentTimeMillis() - stateStartTime;
    }

    private void changeState(ShootState newState) {
        shootState = newState;
        stateStartTime = System.currentTimeMillis();
    }

    public void update() {
        if (!isActive()) return;

        switch (shootState) {
            case ALIGN:
                state.setAutoAlignEnabled(true);
                changeState(ShootState.WAIT_ALIGN);
                debugMessage = "Aligning to tag...";
                break;

            case WAIT_ALIGN:
                if (state.isBasketTagVisible()) {
                    double bearing = state.getTagBearing();
                    debugMessage = String.format("Bearing: %.1f°", bearing);

                    if (Math.abs(bearing) < BEARING_TOLERANCE) {
                        state.setAutoAlignEnabled(false);
                        changeState(ShootState.SPIN_UP);
                        debugMessage = "Aligned!";
                    }
                } else {
                    debugMessage = "Tag not visible, waiting...";
                }

                // Timeout after 3 seconds
                if (getStateTime() > 3000) {
                    state.setAutoAlignEnabled(false);
                    changeState(ShootState.SPIN_UP);
                    debugMessage = "Align timeout";
                }
                break;

            case SPIN_UP:
                if (state.isBasketTagVisible()) {
                    state.setAdjustedVelocity(state.getTagRange());
                    debugMessage = String.format("Spinning up for range %.1f", state.getTagRange());
                } else {
                    state.setShooterTargetVelocity(DEFAULT_VELOCITY);
                    debugMessage = "Using default velocity";
                }
                changeState(ShootState.WAIT_SHOOTER);
                break;

            case WAIT_SHOOTER:
                double currentVel = state.getShooterCurrentVelocity();
                double targetVel = state.getShooterTargetVelocity();
                debugMessage = String.format("Shooter: %.0f / %.0f", currentVel, targetVel);

                if (state.isShooterReady()) {
                    changeState(ShootState.KICK);
                }

                // Timeout after 2 seconds
                if (getStateTime() > 2000) {
                    changeState(ShootState.KICK);
                    debugMessage = "Shooter timeout, kicking anyway";
                }
                break;

            case KICK:
                state.requestKick();
                debugMessage = String.format("Kicking ball %d!", ballsShot + 1);
                changeState(ShootState.WAIT_KICK_DONE);
                break;

            case WAIT_KICK_DONE:
                debugMessage = String.format("Waiting for kick (up=%s, req=%s)",
                        state.isKickerUp(), state.isKickRequested());

                // Wait for kicker to go up and come back down
                if (!state.isKickerUp() && !state.isKickRequested() && getStateTime() > 500) {
                    ballsShot++;
                    debugMessage = String.format("Ball %d shot!", ballsShot);

                    if (ballsShot >= TOTAL_BALLS) {
                        // Done!
                        debugMessage = "All 3 balls shot!";
                        cancel();
                    } else {
                        // Rotate to next ball
                        changeState(ShootState.ROTATE);
                    }
                }
                break;

            case ROTATE:
                // Rotate clockwise (right) to next ball
                state.setCarouselCommand(CarouselCommand.ROTATE_RIGHT);
                debugMessage = "Rotating clockwise...";
                changeState(ShootState.WAIT_ROTATE);
                break;

            case WAIT_ROTATE:
                debugMessage = String.format("Waiting for rotate (settled=%s, time=%dms)",
                        state.isCarouselSettled(), getStateTime());

                // Wait for carousel to settle AND minimum time
                if (state.isCarouselSettled() && getStateTime() > 500) {
                    // Go back to align for next shot
                    changeState(ShootState.ALIGN);
                    debugMessage = "Rotate done, realigning...";
                }

                // Timeout after 2 seconds
                if (getStateTime() > 2000) {
                    changeState(ShootState.ALIGN);
                    debugMessage = "Rotate timeout";
                }
                break;

            case IDLE:
            default:
                break;
        }
    }
}