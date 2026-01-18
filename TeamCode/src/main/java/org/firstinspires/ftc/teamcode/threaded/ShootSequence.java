package org.firstinspires.ftc.teamcode.threaded;

import com.qualcomm.robotcore.util.ElapsedTime;

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

    // Timing constants (milliseconds)
    private static final long CAROUSEL_TIMEOUT_MS = 2000;      // Max wait for carousel
    private static final long SENSOR_SETTLE_MS = 150;          // Wait for sensors after rotation
    private static final long POST_KICK_SETTLE_MS = 500;       // Wait after kick for ball to clear

    // State machine
    private enum ShootState {
        IDLE,
        SPIN_UP,
        WAIT_SHOOTER,
        FIND_BALL,
        COMMAND_CAROUSEL,
        WAIT_CAROUSEL_MOVING,
        WAIT_CAROUSEL_SETTLED,
        WAIT_SENSOR_SETTLE,
        KICK,
        WAIT_KICK_DONE,
        WAIT_POST_KICK
    }

    private ShootState shootState = ShootState.IDLE;
    private BallColor[] activeShootOrder = null;
    private int ballIndex = 0;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private BallColor currentTargetColor = null;

    public ShootSequence(BotState state) {
        this.state = state;
    }

    public void start() {
        if (isActive()) return;

        if (state.hasDetectedShootOrder()) {
            activeShootOrder = state.getDetectedShootOrder();
        } else {
            activeShootOrder = DEFAULT_SHOOT_ORDER;
        }

        ballIndex = 0;
        currentTargetColor = null;
        shootState = ShootState.SPIN_UP;
        stateTimer.reset();
    }

    public void cancel() {
        shootState = ShootState.IDLE;
        ballIndex = 0;
        activeShootOrder = null;
        currentTargetColor = null;
        state.setShooterTargetVelocity(0);
    }

    public boolean isActive() {
        return shootState != ShootState.IDLE;
    }

    public String getStateName() {
        return shootState.toString();
    }

    public int getBallIndex() {
        return ballIndex;
    }

    public int getTotalBalls() {
        return activeShootOrder != null ? activeShootOrder.length : 0;
    }

    public BallColor getCurrentTarget() {
        return currentTargetColor;
    }

    public BallColor[] getShootOrder() {
        return activeShootOrder;
    }

    /**
     * Call this every loop iteration. Does NOT block.
     */
    public void update() {
        if (!isActive()) return;

        switch (shootState) {
            case SPIN_UP:
                // Set shooter velocity based on distance
                if (state.isBasketTagVisible()) {
                    state.setAdjustedVelocity(state.getTagRange());
                } else {
                    state.setShooterTargetVelocity(DEFAULT_VELOCITY);
                }
                shootState = ShootState.WAIT_SHOOTER;
                stateTimer.reset();
                break;

            case WAIT_SHOOTER:
                if (state.isShooterReady()) {
                    shootState = ShootState.FIND_BALL;
                }
                break;

            case FIND_BALL:
                // Check if we're done
                if (ballIndex >= activeShootOrder.length) {
                    cancel();
                    return;
                }

                currentTargetColor = activeShootOrder[ballIndex];

                // Check if we have this color
                if (!state.hasColor(currentTargetColor)) {
                    // Don't have this color, skip to next
                    ballIndex++;
                    return;
                }

                // Check if already at intake position
                if (state.getPositionColor(BotState.POS_INTAKE) == currentTargetColor) {
                    // Already in position, go to kick
                    shootState = ShootState.KICK;
                } else {
                    // Need to rotate
                    shootState = ShootState.COMMAND_CAROUSEL;
                }
                break;

            case COMMAND_CAROUSEL:
                // Use the correct command based on target color
                if (currentTargetColor == BallColor.GREEN) {
                    state.setCarouselCommand(CarouselCommand.ROTATE_TO_KICK_GREEN);
                } else {
                    state.setCarouselCommand(CarouselCommand.ROTATE_TO_KICK_PURPLE);
                }
                shootState = ShootState.WAIT_CAROUSEL_MOVING;
                stateTimer.reset();
                break;

            case WAIT_CAROUSEL_MOVING:
                // Wait for carousel to start moving (no longer settled)
                // Or if command was already processed and it's a no-op (already settled)
                if (!state.isCarouselSettled()) {
                    // Carousel started moving
                    shootState = ShootState.WAIT_CAROUSEL_SETTLED;
                    stateTimer.reset();
                } else if (stateTimer.milliseconds() > 100) {
                    // If still settled after 100ms, command was probably a no-op
                    // (ball was already at intake or very close)
                    shootState = ShootState.WAIT_SENSOR_SETTLE;
                    stateTimer.reset();
                }
                break;

            case WAIT_CAROUSEL_SETTLED:
                // Wait for carousel to finish moving
                if (state.isCarouselSettled()) {
                    shootState = ShootState.WAIT_SENSOR_SETTLE;
                    stateTimer.reset();
                } else if (stateTimer.milliseconds() > CAROUSEL_TIMEOUT_MS) {
                    // Timeout - something went wrong, try to continue anyway
                    shootState = ShootState.WAIT_SENSOR_SETTLE;
                    stateTimer.reset();
                }
                break;

            case WAIT_SENSOR_SETTLE:
                // Give sensors time to read the new ball positions
                if (stateTimer.milliseconds() >= SENSOR_SETTLE_MS) {
                    // Verify the ball is actually at intake now
                    if (state.getPositionColor(BotState.POS_INTAKE) == currentTargetColor) {
                        shootState = ShootState.KICK;
                    } else if (state.hasColor(currentTargetColor)) {
                        // Ball exists but not at intake - try rotating again
                        shootState = ShootState.COMMAND_CAROUSEL;
                    } else {
                        // Ball is gone somehow, skip to next
                        ballIndex++;
                        shootState = ShootState.FIND_BALL;
                    }
                }
                break;

            case KICK:
                // Update velocity in case distance changed
                if (state.isBasketTagVisible()) {
                    state.setAdjustedVelocity(state.getTagRange());
                }

                // Wait for shooter to be ready, then kick
                if (state.isShooterReady() && state.isCarouselSettled()) {
                    state.requestKick();
                    shootState = ShootState.WAIT_KICK_DONE;
                    stateTimer.reset();
                }
                break;

            case WAIT_KICK_DONE:
                // Wait for kicker to return down
                if (!state.isKickerUp() && !state.isKickRequested()) {
                    shootState = ShootState.WAIT_POST_KICK;
                    stateTimer.reset();
                }
                break;

            case WAIT_POST_KICK:
                // Wait for ball to clear and sensors to update
                if (stateTimer.milliseconds() >= POST_KICK_SETTLE_MS) {
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