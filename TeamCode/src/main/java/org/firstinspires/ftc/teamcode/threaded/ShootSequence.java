package org.firstinspires.ftc.teamcode.threaded;

import org.firstinspires.ftc.teamcode.threaded.BotState.BallColor;
import org.firstinspires.ftc.teamcode.threaded.BotState.CarouselCommand;

public class ShootSequence {

    private final BotState state;

    // Default shoot order if no tag detected
    private static final BallColor[] DEFAULT_SHOOT_ORDER = {
            BallColor.GREEN,
            BallColor.PURPLE,
            BallColor.PURPLE
    };

    // Default velocity if tag not visible
    private static final double DEFAULT_VELOCITY = 210.0;

    // State machine
    private enum ShootState {
        IDLE,
        SPIN_UP,           // Spin up shooter based on distance
        WAIT_SHOOTER,      // Wait for shooter to reach speed
        FIND_BALL,         // Find the ball and command rotation
        WAIT_CAROUSEL,     // Wait for carousel to settle
        KICK,              // Kick the ball
        WAIT_KICK_DONE     // Wait for kick to finish
    }

    private ShootState shootState = ShootState.IDLE;
    private BallColor[] activeShootOrder = null;
    private int ballIndex = 0;

    public ShootSequence(BotState state) {
        this.state = state;
    }

    /**
     * Start the shoot sequence.
     * Uses detected shoot order from AprilTag if available, otherwise uses default.
     */
    public void start() {
        if (isActive()) return;

        // Get shoot order from detected tag or use default
        if (state.hasDetectedShootOrder()) {
            activeShootOrder = state.getDetectedShootOrder();
        } else {
            activeShootOrder = DEFAULT_SHOOT_ORDER;
        }

        ballIndex = 0;
        shootState = ShootState.SPIN_UP;
    }

    /**
     * Cancel the shoot sequence and stop the shooter.
     */
    public void cancel() {
        shootState = ShootState.IDLE;
        ballIndex = 0;
        activeShootOrder = null;
        state.setShooterTargetVelocity(0);
    }

    /**
     * Check if shoot sequence is currently running.
     */
    public boolean isActive() {
        return shootState != ShootState.IDLE;
    }

    /**
     * Get current state for telemetry.
     */
    public String getStateName() {
        return shootState.toString();
    }

    /**
     * Get current ball index (0-based).
     */
    public int getBallIndex() {
        return ballIndex;
    }

    /**
     * Get total balls in sequence.
     */
    public int getTotalBalls() {
        return activeShootOrder != null ? activeShootOrder.length : 0;
    }

    /**
     * Get current target color.
     */
    public BallColor getCurrentTarget() {
        if (activeShootOrder != null && ballIndex < activeShootOrder.length) {
            return activeShootOrder[ballIndex];
        }
        return null;
    }

    /**
     * Get the active shoot order.
     */
    public BallColor[] getShootOrder() {
        return activeShootOrder;
    }

    /**
     * Call this every loop iteration to run the state machine.
     */
    public void update() throws InterruptedException {
        if (!isActive()) return;

        switch (shootState) {
            case SPIN_UP:
                // Get distance and set shooter velocity
                if (state.isBasketTagVisible()) {
                    state.setAdjustedVelocity(state.getTagRange());
                } else {
                    state.setShooterTargetVelocity(DEFAULT_VELOCITY);
                }
                shootState = ShootState.WAIT_SHOOTER;
                break;

            case WAIT_SHOOTER:
                // Wait for shooter to reach speed before starting
                if (state.isShooterReady()) {
                    shootState = ShootState.FIND_BALL;
                }
                break;

            case FIND_BALL:
                // Check if we're done with all balls
                if (ballIndex >= activeShootOrder.length) {
                    cancel();
                    return;
                }

                BallColor targetColor = activeShootOrder[ballIndex];

                // Check if we have this color
                if (!state.hasColor(targetColor)) {
                    // Skip this ball, move to next
                    ballIndex++;
                    // Stay in FIND_BALL state to process next
                    return;
                }

                // Find where this ball is and rotate it to intake
                int ballPosition = state.findPositionWithColor(targetColor);
                if (ballPosition == BotState.POS_INTAKE) {
                    // Already at kick position, go straight to kick
                    shootState = ShootState.KICK;
                } else if (ballPosition == BotState.POS_BACK_LEFT) {
                    state.setCarouselCommand(CarouselCommand.ROTATE_LEFT);
                    shootState = ShootState.WAIT_CAROUSEL;
                } else if (ballPosition == BotState.POS_BACK_RIGHT) {
                    state.setCarouselCommand(CarouselCommand.ROTATE_RIGHT);
                    shootState = ShootState.WAIT_CAROUSEL;
                }
                break;

            case WAIT_CAROUSEL:
                // Wait for carousel to finish moving
                Thread.sleep(1000);
                shootState = ShootState.KICK;
                break;

            case KICK:
                // Make sure shooter is still ready, then kick
                if (state.isShooterReady()) {
                    state.requestKick();
                    shootState = ShootState.WAIT_KICK_DONE;
                }
                break;

            case WAIT_KICK_DONE:
                // Wait for kick to complete
                if (!state.isKickerUp() && !state.isKickRequested()) {
                    Thread.sleep(1000);
                    // Move to next ball
                    ballIndex++;
                    shootState = ShootState.FIND_BALL;
                }
                break;

            case IDLE:
            default:
                break;
        }
    }
}