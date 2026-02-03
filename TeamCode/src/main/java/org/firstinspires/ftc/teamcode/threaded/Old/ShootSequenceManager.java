package org.firstinspires.ftc.teamcode.threaded.Old;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.threaded.Old.BotState.BallColor;
import org.firstinspires.ftc.teamcode.threaded.Old.BotState.CarouselCommand;

/**
 * Manages automated shooting sequences.
 *
 * Simplified version using kicker voltage feedback instead of timing delays.
 * Carousel movement is blocked until kicker voltage indicates it's down.
 *
 * Usage:
 *   ShootSequenceManager seq = new ShootSequenceManager();
 *
 *   // Full 3-ball sequence:
 *   seq.start(state, targetOrder);
 *
 *   // Single ball:
 *   seq.shootSingle(state, BallColor.GREEN);
 *   seq.shootSingle(state, BallColor.PURPLE);
 *
 *   // Call every loop:
 *   seq.update(state);
 */
public class ShootSequenceManager {

    // ========================= STATE MACHINE =========================
    public enum State {
        IDLE,           // Waiting for start
        SPIN_UP,        // Waiting for shooter velocity
        ROTATING,       // Issuing carousel rotation
        WAIT_SETTLE,    // Waiting for carousel to finish AND kicker to be down
        KICKING,        // Kick in progress, waiting for kicker to return down
        COMPLETE,       // All done
        ERROR           // Something went wrong
    }

    private State state = State.IDLE;
    private final ElapsedTime timer = new ElapsedTime();

    // The plan
    private int[] rotationPlan = null;
    private BallColor[] targetOrder = null;
    private int ballIndex = 0;

    // Debug info
    private String errorMessage = "";
    private BallColor[] startingPositions = null;

    // ========================= PUBLIC METHODS =========================

    /**
     * Start the shooting sequence.
     * @param botState Current BotState
     * @param targetOrder Array of 3 colors in the order to shoot them
     * @return true if sequence started, false if invalid
     */
    public boolean start(BotState botState, BallColor[] targetOrder) {
        if (targetOrder == null || targetOrder.length != 3) {
            errorMessage = "Invalid target order";
            this.state = State.ERROR;
            return false;
        }

        startingPositions = botState.getAllPositions();

        int ballCount = 0;
        for (BallColor c : startingPositions) {
            if (c == BallColor.GREEN || c == BallColor.PURPLE) ballCount++;
        }
        if (ballCount < 3) {
            errorMessage = "Only " + ballCount + " balls detected";
            this.state = State.ERROR;
            return false;
        }

        this.targetOrder = targetOrder.clone();
        this.rotationPlan = calculatePlan(startingPositions, targetOrder);

        if (rotationPlan == null) {
            errorMessage = "Could not create valid plan";
            this.state = State.ERROR;
            return false;
        }

        ballIndex = 0;
        this.state = State.SPIN_UP;
        timer.reset();

        return true;
    }

    /**
     * Shoot a single ball of the specified color.
     */
    public boolean shootSingle(BotState botState, BallColor color) {
        startingPositions = botState.getAllPositions();

        BallColor intake = startingPositions[0];
        BallColor backLeft = startingPositions[1];
        BallColor backRight = startingPositions[2];

        int rotation;
        if (intake == color) {
            rotation = 0;
        } else if (backLeft == color) {
            rotation = 1;
        } else if (backRight == color) {
            rotation = -1;
        } else {
            errorMessage = color + " not found";
            this.state = State.ERROR;
            return false;
        }

        this.rotationPlan = new int[] { rotation };
        this.targetOrder = new BallColor[] { color };

        ballIndex = 0;
        this.state = State.SPIN_UP;
        timer.reset();

        return true;
    }

    /**
     * Call this every loop to run the state machine.
     */
    public void update(BotState botState) {
        switch (this.state) {
            case IDLE:
            case COMPLETE:
            case ERROR:
                break;

            case SPIN_UP:
                handleSpinUp(botState);
                break;

            case ROTATING:
                handleRotating(botState);
                break;

            case WAIT_SETTLE:
                handleWaitSettle(botState);
                break;

            case KICKING:
                handleKicking(botState);
                break;
        }
    }

    /**
     * Abort the sequence immediately.
     */
    public void abort() {
        state = State.IDLE;
        rotationPlan = null;
        targetOrder = null;
        ballIndex = 0;
    }

    /**
     * Check if sequence is currently running.
     */
    public boolean isRunning() {
        return state != State.IDLE && state != State.COMPLETE && state != State.ERROR;
    }

    // ========================= STATE HANDLERS =========================

    private void handleSpinUp(BotState botState) {
        // Wait for shooter to be ready OR timeout
        if (botState.isShooterReady() || timer.milliseconds() >= BotState.SEQ_SPIN_UP_MS) {
            this.state = State.ROTATING;
            timer.reset();
        }
    }

    private void handleRotating(BotState botState) {
        int rotation = rotationPlan[ballIndex];

        if (rotation == 0) {
            // No rotation needed, go straight to waiting for settle/kicker
            this.state = State.WAIT_SETTLE;
            timer.reset();
        } else if (rotation == -1) {
            botState.setCarouselCommand(CarouselCommand.ROTATE_LEFT);
            this.state = State.WAIT_SETTLE;
            timer.reset();
        } else if (rotation == 1) {
            botState.setCarouselCommand(CarouselCommand.ROTATE_RIGHT);
            this.state = State.WAIT_SETTLE;
            timer.reset();
        }
    }

    private void handleWaitSettle(BotState botState) {
        // Wait for carousel to settle AND kicker to be down
        if (botState.isCarouselSettled() && botState.isKickerDown()) {
            // Ready to kick
            botState.requestKick();
            this.state = State.KICKING;
            timer.reset();
        }
    }

    private void handleKicking(BotState botState) {
        // Wait for kicker to return to down position (voltage feedback)
        // OR timeout as safety
        boolean kickerReturned = botState.isKickerDown() && timer.milliseconds() > 100; // Min 100ms to avoid false trigger
        boolean timeout = timer.milliseconds() >= BotState.SEQ_KICK_TIMEOUT_MS;

        if (kickerReturned || timeout) {
            // Move to next ball
            ballIndex++;

            if (ballIndex >= rotationPlan.length) {
                this.state = State.COMPLETE;
            } else {
                // Next shot
                this.state = State.ROTATING;
                timer.reset();
            }
        }
    }

    // ========================= PLANNING LOGIC =========================

    private int[] calculatePlan(BallColor[] positions, BallColor[] targetOrder) {
        BallColor intake = positions[0];
        BallColor backLeft = positions[1];
        BallColor backRight = positions[2];

        int[] plan = new int[3];

        for (int shot = 0; shot < 3; shot++) {
            BallColor need = targetOrder[shot];

            if (intake == need) {
                plan[shot] = 0;
            } else if (backLeft == need) {
                plan[shot] = 1;
                BallColor temp = intake;
                intake = backLeft;
                backLeft = backRight;
                backRight = temp;
            } else if (backRight == need) {
                plan[shot] = -1;
                BallColor temp = intake;
                intake = backRight;
                backRight = backLeft;
                backLeft = temp;
            } else {
                return null;
            }

            intake = BallColor.EMPTY;
        }

        return plan;
    }

    // ========================= DEBUG / TELEMETRY =========================

    public State getState() {
        return state;
    }

    public int getBallIndex() {
        return ballIndex;
    }

    public long getElapsedMs() {
        return (long) timer.milliseconds();
    }

    public String getErrorMessage() {
        return errorMessage;
    }

    public int[] getRotationPlan() {
        return rotationPlan;
    }

    public BallColor[] getTargetOrder() {
        return targetOrder;
    }

    public BallColor[] getStartingPositions() {
        return startingPositions;
    }

    public String getDebugString() {
        StringBuilder sb = new StringBuilder();
        sb.append("State: ").append(state);

        if (rotationPlan != null) {
            sb.append(" | Shot: ").append(ballIndex + 1).append("/").append(rotationPlan.length);
        }
        sb.append(" | Timer: ").append(getElapsedMs()).append("ms");

        if (rotationPlan != null) {
            sb.append("\nPlan: [");
            for (int i = 0; i < rotationPlan.length; i++) {
                if (i == ballIndex && isRunning()) sb.append(">");
                sb.append(rotationPlan[i]);
                if (i < rotationPlan.length - 1) sb.append(", ");
            }
            sb.append("]");
        }

        if (state == State.ERROR) {
            sb.append("\nERROR: ").append(errorMessage);
        }

        return sb.toString();
    }
}