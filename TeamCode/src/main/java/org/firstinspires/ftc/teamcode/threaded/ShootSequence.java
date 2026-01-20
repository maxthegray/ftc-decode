package org.firstinspires.ftc.teamcode.threaded;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.threaded.BotState.BallColor;
import org.firstinspires.ftc.teamcode.threaded.BotState.CarouselCommand;

/**
 * Simple state-machine command to shoot balls in a specific order.
 *
 * Usage:
 *   // At init:
 *   ShootSequence shootSequence = new ShootSequence(state);
 *
 *   // On button press:
 *   shootSequence.start(new BallColor[]{GREEN, PURPLE, PURPLE}, 210.0);
 *
 *   // In loop:
 *   shootSequence.update();
 */
public class ShootSequence {

    // ========================= TUNABLE DELAYS (ms) =========================
    public static long DELAY_START_MS = 200;         // Before first rotation
    public static long DELAY_BEFORE_KICK_MS = 150;   // After settle, before kick
    public static long DELAY_AFTER_KICK_MS = 200;    // After kick, before next rotation

    // ========================= STATE MACHINE =========================
    private enum State {
        IDLE,
        START_DELAY,
        ROTATE,
        WAIT_SETTLE,
        PRE_KICK_DELAY,
        KICK,
        WAIT_KICK_DONE,
        POST_KICK_DELAY,
        COMPLETE
    }

    private final BotState botState;

    private State currentState = State.IDLE;
    private BallColor[] shootOrder = null;
    private double lockedVelocity = 0;
    private int currentBallIndex = 0;
    private final ElapsedTime timer = new ElapsedTime();

    // Track shots
    private int shotsFired = 0;
    private int shotsSkipped = 0;

    public ShootSequence(BotState botState) {
        this.botState = botState;
    }

    /**
     * Start the shoot sequence with specified order and velocity
     */
    public void start(BallColor[] order, double velocity) {
        if (currentState != State.IDLE && currentState != State.COMPLETE) return;

        this.shootOrder = order;
        this.lockedVelocity = velocity;

        // Lock in shooter velocity
        botState.setShooterTargetVelocity(lockedVelocity);

        // Reset counters
        currentBallIndex = 0;
        shotsFired = 0;
        shotsSkipped = 0;

        // Begin
        currentState = State.START_DELAY;
        timer.reset();
    }

    /**
     * Call this every loop iteration
     */
    public void update() {
        if (shootOrder == null) return;

        switch (currentState) {
            case IDLE:
            case COMPLETE:
                // Nothing to do
                break;

            case START_DELAY:
                if (timer.milliseconds() >= DELAY_START_MS) {
                    advanceToNextBall();
                }
                break;

            case ROTATE:
                // Command was sent, move to waiting
                currentState = State.WAIT_SETTLE;
                break;

            case WAIT_SETTLE:
                if (botState.isCarouselSettled()) {
                    currentState = State.PRE_KICK_DELAY;
                    timer.reset();
                }
                break;

            case PRE_KICK_DELAY:
                if (timer.milliseconds() >= DELAY_BEFORE_KICK_MS) {
                    // Verify shooter is ready before kicking
                    if (botState.isShooterReady()) {
                        botState.requestKick();
                        currentState = State.KICK;
                    }
                    // If not ready, stay in this state until it is
                }
                break;

            case KICK:
                // Wait for kick to start
                if (botState.isKickerUp()) {
                    currentState = State.WAIT_KICK_DONE;
                }
                break;

            case WAIT_KICK_DONE:
                if (!botState.isKickerUp()) {
                    shotsFired++;
                    currentState = State.POST_KICK_DELAY;
                    timer.reset();
                }
                break;

            case POST_KICK_DELAY:
                if (timer.milliseconds() >= DELAY_AFTER_KICK_MS) {
                    currentBallIndex++;
                    advanceToNextBall();
                }
                break;
        }
    }

    /**
     * Find and rotate to the next ball, or complete if done
     */
    private void advanceToNextBall() {
        // Skip any balls not present in carousel
        while (currentBallIndex < shootOrder.length) {
            BallColor targetColor = shootOrder[currentBallIndex];

            // Check if we have this color
            if (botState.hasColor(targetColor)) {
                // Send rotate command
                if (targetColor == BallColor.GREEN) {
                    botState.setCarouselCommand(CarouselCommand.ROTATE_TO_KICK_GREEN);
                } else if (targetColor == BallColor.PURPLE) {
                    botState.setCarouselCommand(CarouselCommand.ROTATE_TO_KICK_PURPLE);
                }
                currentState = State.ROTATE;
                return;
            } else {
                // Skip this ball
                shotsSkipped++;
                currentBallIndex++;
            }
        }

        // No more balls to shoot
        currentState = State.COMPLETE;
        botState.setShooterTargetVelocity(0);
    }

    // ========================= STATUS METHODS =========================

    public boolean isActive() {
        return currentState != State.IDLE && currentState != State.COMPLETE;
    }

    public boolean isComplete() {
        return currentState == State.COMPLETE;
    }

    public String getStateName() {
        return currentState.toString();
    }

    public int getBallIndex() {
        return currentBallIndex;
    }

    public int getTotalBalls() {
        return (shootOrder != null) ? shootOrder.length : 0;
    }

    public BallColor getCurrentTarget() {
        if (shootOrder != null && currentBallIndex < shootOrder.length) {
            return shootOrder[currentBallIndex];
        }
        return null;
    }

    public BallColor[] getShootOrder() {
        return shootOrder;
    }

    public int getShotsFired() {
        return shotsFired;
    }

    public int getShotsSkipped() {
        return shotsSkipped;
    }

    /**
     * Reset to idle state (for reuse)
     */
    public void reset() {
        currentState = State.IDLE;
        shootOrder = null;
        currentBallIndex = 0;
        shotsFired = 0;
        shotsSkipped = 0;
    }
}