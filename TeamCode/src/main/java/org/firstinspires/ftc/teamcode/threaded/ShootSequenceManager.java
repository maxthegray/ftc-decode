package org.firstinspires.ftc.teamcode.threaded;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.threaded.BotState.BallColor;
import org.firstinspires.ftc.teamcode.threaded.BotState.CarouselCommand;

/**
 * Manages automated shooting sequences.
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
 *
 * Timing constants are in BotState for easy Dashboard tuning.
 */
public class ShootSequenceManager {

    // ========================= STATE MACHINE =========================
    public enum State {
        IDLE,           // Waiting for start
        SPIN_UP,        // Waiting for shooter velocity
        ROTATING,       // Issuing carousel rotation
        WAIT_SETTLE,    // Waiting for carousel to finish
        CAROUSEL_DELAY, // Delay after carousel settles
        KICKING,        // Kick in progress
        POST_KICK,      // Delay after kick
        COMPLETE,       // All done
        ERROR           // Something went wrong
    }

    private State state = State.IDLE;
    private final ElapsedTime timer = new ElapsedTime();

    // The plan
    private int[] rotationPlan = null;      // e.g., [-1, -1, +1]
    private BallColor[] targetOrder = null; // e.g., [GREEN, PURPLE, PURPLE]
    private int ballIndex = 0;              // 0, 1, 2

    // Debug info
    private String errorMessage = "";
    private BallColor[] startingPositions = null;

    // ========================= PUBLIC METHODS =========================

    /**
     * Start the shooting sequence.
     * @param state Current BotState
     * @param targetOrder Array of 3 colors in the order to shoot them
     * @return true if sequence started, false if invalid
     */
    public boolean start(BotState state, BallColor[] targetOrder) {
        // Validate input
        if (targetOrder == null || targetOrder.length != 3) {
            errorMessage = "Invalid target order";
            this.state = State.ERROR;
            return false;
        }

        // Snapshot current positions
        startingPositions = state.getAllPositions();

        // Validate we have 3 balls
        int ballCount = 0;
        for (BallColor c : startingPositions) {
            if (c == BallColor.GREEN || c == BallColor.PURPLE) ballCount++;
        }
        if (ballCount < 3) {
            errorMessage = "Only " + ballCount + " balls detected";
            this.state = State.ERROR;
            return false;
        }

        // Calculate the rotation plan
        this.targetOrder = targetOrder.clone();
        this.rotationPlan = calculatePlan(startingPositions, targetOrder);

        if (rotationPlan == null) {
            errorMessage = "Could not create valid plan";
            this.state = State.ERROR;
            return false;
        }

        // Start the sequence
        ballIndex = 0;
        this.state = State.SPIN_UP;
        timer.reset();

        return true;
    }

    /**
     * Shoot a single ball of the specified color.
     * Finds the ball, rotates it to intake, and kicks it.
     *
     * @param state Current BotState
     * @param color The color to shoot (GREEN or PURPLE)
     * @return true if started, false if color not found
     */
    public boolean shootSingle(BotState state, BallColor color) {
        // Snapshot current positions
        startingPositions = state.getAllPositions();

        // Find where the color is
        BallColor intake    = startingPositions[0];
        BallColor backLeft  = startingPositions[1];
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

        // Create single-shot plan
        this.rotationPlan = new int[] { rotation };
        this.targetOrder = new BallColor[] { color };

        // Start the sequence
        ballIndex = 0;
        this.state = State.SPIN_UP;
        timer.reset();

        return true;
    }

    /**
     * Call this every loop to run the state machine.
     */
    public void update(BotState state) {
        switch (this.state) {
            case IDLE:
            case COMPLETE:
            case ERROR:
                // Nothing to do
                break;

            case SPIN_UP:
                handleSpinUp(state);
                break;

            case ROTATING:
                handleRotating(state);
                break;

            case WAIT_SETTLE:
                handleWaitSettle(state);
                break;

            case CAROUSEL_DELAY:
                handleCarouselDelay(state);
                break;

            case KICKING:
                handleKicking(state);
                break;

            case POST_KICK:
                handlePostKick(state);
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

    private void handleSpinUp(BotState state) {
        // Wait for shooter to be ready OR timeout
        if (state.isShooterReady() || timer.milliseconds() >= BotState.SEQ_SPIN_UP_MS) {
            // Move to rotating
            this.state = State.ROTATING;
            timer.reset();
        }
    }

    private void handleRotating(BotState state) {
        // Issue the rotation command
        int rotation = rotationPlan[ballIndex];

        if (rotation == 0) {
            // No rotation needed, skip to carousel delay (which will immediately transition to kick)
            this.state = State.CAROUSEL_DELAY;
            timer.reset();
        } else if (rotation == -1) {
            state.setCarouselCommand(CarouselCommand.ROTATE_LEFT);
            this.state = State.WAIT_SETTLE;
        } else if (rotation == 1) {
            state.setCarouselCommand(CarouselCommand.ROTATE_RIGHT);
            this.state = State.WAIT_SETTLE;
        }
    }

    private void handleWaitSettle(BotState state) {
        // Wait for carousel to report settled
        if (state.isCarouselSettled()) {
            // Carousel just settled, start the delay timer
            this.state = State.CAROUSEL_DELAY;
            timer.reset();
        }
    }

    private void handleCarouselDelay(BotState state) {
        // Wait for delay after carousel settles
        if (timer.milliseconds() >= BotState.SEQ_CAROUSEL_DELAY_MS) {
            // Request kick
            state.requestKick();
            this.state = State.KICKING;
            timer.reset();
        }
    }

    private void handleKicking(BotState state) {
        // Wait for kick to complete
        if (timer.milliseconds() >= BotState.SEQ_KICK_MS) {
            this.state = State.POST_KICK;
            timer.reset();
        }
    }

    private void handlePostKick(BotState state) {
        // Wait for post-kick delay
        if (timer.milliseconds() >= BotState.SEQ_POST_KICK_MS) {
            // Move to next ball
            ballIndex++;

            if (ballIndex >= rotationPlan.length) {
                // All done!
                this.state = State.COMPLETE;
            } else {
                // Next shot - go back to rotating
                this.state = State.ROTATING;
                timer.reset();
            }
        }
    }

    // ========================= PLANNING LOGIC =========================

    /**
     * Calculate the rotation plan for shooting 3 balls in a specific order.
     *
     * @param positions   Current balls: [intake, backLeft, backRight]
     * @param targetOrder Desired shoot order: [first, second, third]
     * @return Rotations for each shot: -1=left, 0=none, +1=right, or null if impossible
     */
    private int[] calculatePlan(BallColor[] positions, BallColor[] targetOrder) {

        // Working state - what's in each position right now?
        BallColor intake    = positions[0];
        BallColor backLeft  = positions[1];
        BallColor backRight = positions[2];

        int[] plan = new int[3];

        for (int shot = 0; shot < 3; shot++) {

            BallColor need = targetOrder[shot];

            // Where is the ball we need?
            if (intake == need) {
                // Already at intake - no rotation needed
                plan[shot] = 0;

            } else if (backLeft == need) {
                // Rotate LEFT to bring backLeft to intake
                plan[shot] = 1;
                BallColor temp = intake;
                intake = backLeft;
                backLeft = backRight;
                backRight = temp;

            } else if (backRight == need) {
                // Rotate RIGHT to bring backRight to intake
                plan[shot] = -1;
                BallColor temp = intake;
                intake = backRight;
                backRight = backLeft;
                backLeft = temp;

            } else {
                // Ball not found - impossible plan
                return null;
            }

            // Kick the ball (intake becomes empty)
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

    /**
     * Get a debug string for telemetry.
     */
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

    /**
     * Get detailed plan info for debugging.
     */
    public String getPlanDebugString() {
        if (startingPositions == null || targetOrder == null || rotationPlan == null) {
            return "No plan";
        }

        StringBuilder sb = new StringBuilder();
        sb.append("Starting: [")
                .append(startingPositions[0]).append(", ")
                .append(startingPositions[1]).append(", ")
                .append(startingPositions[2]).append("]\n");

        sb.append("Target:   [");
        for (int i = 0; i < targetOrder.length; i++) {
            sb.append(targetOrder[i]);
            if (i < targetOrder.length - 1) sb.append(", ");
        }
        sb.append("]\n");

        sb.append("Plan:     [");
        for (int i = 0; i < rotationPlan.length; i++) {
            sb.append(rotationPlan[i]);
            if (i < rotationPlan.length - 1) sb.append(", ");
        }
        sb.append("]");

        return sb.toString();
    }
}