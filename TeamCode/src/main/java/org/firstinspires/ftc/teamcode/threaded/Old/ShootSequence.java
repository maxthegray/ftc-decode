package org.firstinspires.ftc.teamcode.threaded.Old;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Shoot sequence state machine.
 *
 * KEY DESIGN: This class does NOT touch hardware. It only:
 *   1. Tracks state
 *   2. Tells you what action to take via getNextAction()
 *   3. You tell it when actions complete via confirmXxx()
 *
 * This makes it easy to test and debug - the logic is pure.
 */
public class ShootSequence {

    // ==================== TYPES ====================

    public enum BallColor { GREEN, PURPLE, EMPTY, UNKNOWN }

    public enum State {
        IDLE,
        WAIT_SHOOTER,      // Waiting for shooter to spin up
        NEED_ROTATE,       // Need to rotate carousel
        WAIT_ROTATE,       // Waiting for carousel to settle
        NEED_KICK,         // Ready to kick
        WAIT_KICK,         // Waiting for kick to complete
        DONE,
        ERROR
    }

    /**
     * Actions the coordinator should take.
     */
    public enum Action {
        NONE,              // Do nothing
        ROTATE_LEFT,       // Rotate carousel left 1 slot
        ROTATE_RIGHT,      // Rotate carousel right 1 slot
        KICK               // Fire the kicker
    }

    // ==================== STATE ====================

    private State state = State.IDLE;
    private int[] rotationPlan = null;
    private int currentShot = 0;
    private String errorMessage = "";
    private final ElapsedTime timer = new ElapsedTime();

    private static final long SHOOTER_TIMEOUT_MS = 1500;

    // ==================== PUBLIC API ====================

    /**
     * Start a full 3-ball sequence.
     */
    public boolean start(BallColor[] positions, BallColor[] targetOrder) {
        if (positions == null || positions.length != 3) {
            errorMessage = "Invalid positions";
            state = State.ERROR;
            return false;
        }
        if (targetOrder == null || targetOrder.length != 3) {
            errorMessage = "Invalid target order";
            state = State.ERROR;
            return false;
        }

        rotationPlan = calculatePlan(positions, targetOrder);
        if (rotationPlan == null) {
            errorMessage = "Cannot create plan for given positions/order";
            state = State.ERROR;
            return false;
        }

        currentShot = 0;
        errorMessage = "";
        state = State.WAIT_SHOOTER;
        timer.reset();
        return true;
    }

    /**
     * Start a single-ball shot.
     */
    public boolean startSingle(BallColor[] positions, BallColor color) {
        int rotation = findRotationForColor(positions, color);
        if (rotation == Integer.MIN_VALUE) {
            errorMessage = color + " not found";
            state = State.ERROR;
            return false;
        }

        rotationPlan = new int[] { rotation };
        currentShot = 0;
        errorMessage = "";
        state = State.WAIT_SHOOTER;
        timer.reset();
        return true;
    }

    /**
     * Get the next action to perform. Call this each loop.
     * @param shooterReady true if shooter is at target velocity
     */
    public Action getNextAction(boolean shooterReady) {
        switch (state) {
            case WAIT_SHOOTER:
                if (shooterReady || timer.milliseconds() >= SHOOTER_TIMEOUT_MS) {
                    return advanceToRotateOrKick();
                }
                return Action.NONE;

            case NEED_ROTATE:
                int rotation = rotationPlan[currentShot];
                state = State.WAIT_ROTATE;
                if (rotation == 1) return Action.ROTATE_RIGHT;
                if (rotation == -1) return Action.ROTATE_LEFT;
                // rotation == 0, skip to kick
                state = State.NEED_KICK;
                return Action.NONE;

            case NEED_KICK:
                state = State.WAIT_KICK;
                timer.reset();
                return Action.KICK;

            default:
                return Action.NONE;
        }
    }

    /**
     * Call when carousel rotation completes.
     */
    public void confirmRotateComplete() {
        if (state == State.WAIT_ROTATE) {
            state = State.NEED_KICK;
        }
    }

    /**
     * Call when kick completes (kicker back down).
     */
    public void confirmKickComplete() {
        if (state == State.WAIT_KICK) {
            currentShot++;
            if (currentShot >= rotationPlan.length) {
                state = State.DONE;
            } else {
                // Next shot
                state = State.NEED_ROTATE;
            }
        }
    }

    /**
     * Abort the sequence.
     */
    public void abort() {
        state = State.IDLE;
        rotationPlan = null;
        currentShot = 0;
    }

    /**
     * Reset after completion or error.
     */
    public void reset() {
        state = State.IDLE;
        rotationPlan = null;
        currentShot = 0;
        errorMessage = "";
    }

    // ==================== QUERIES ====================

    public boolean isRunning() {
        return state != State.IDLE && state != State.DONE && state != State.ERROR;
    }

    public boolean isDone() {
        return state == State.DONE;
    }

    public boolean isError() {
        return state == State.ERROR;
    }

    public State getState() {
        return state;
    }

    public String getErrorMessage() {
        return errorMessage;
    }

    public int getCurrentShot() {
        return currentShot;
    }

    public int getTotalShots() {
        return rotationPlan != null ? rotationPlan.length : 0;
    }

    public String getDebugString() {
        if (state == State.IDLE) return "IDLE";
        if (state == State.ERROR) return "ERROR: " + errorMessage;
        if (state == State.DONE) return "DONE";

        return String.format("%s | Shot %d/%d | Plan: %s",
                state.name(),
                currentShot + 1,
                rotationPlan != null ? rotationPlan.length : 0,
                planToString());
    }

    // ==================== INTERNAL ====================

    private Action advanceToRotateOrKick() {
        int rotation = rotationPlan[currentShot];
        if (rotation == 0) {
            state = State.NEED_KICK;
        } else {
            state = State.NEED_ROTATE;
        }
        return Action.NONE;
    }

    private int findRotationForColor(BallColor[] positions, BallColor color) {
        if (positions[0] == color) return 0;
        if (positions[1] == color) return 1;   // Back-left → rotate right
        if (positions[2] == color) return -1;  // Back-right → rotate left
        return Integer.MIN_VALUE;  // Not found
    }

    private int[] calculatePlan(BallColor[] positions, BallColor[] targetOrder) {
        // Work with copies
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
                // Simulate rotation right
                BallColor temp = intake;
                intake = backLeft;
                backLeft = backRight;
                backRight = temp;
            } else if (backRight == need) {
                plan[shot] = -1;
                // Simulate rotation left
                BallColor temp = intake;
                intake = backRight;
                backRight = backLeft;
                backLeft = temp;
            } else {
                return null;  // Can't find needed color
            }

            // Ball shot, intake now empty
            intake = BallColor.EMPTY;
        }

        return plan;
    }

    private String planToString() {
        if (rotationPlan == null) return "[]";
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < rotationPlan.length; i++) {
            if (i == currentShot) sb.append(">");
            sb.append(rotationPlan[i]);
            if (i < rotationPlan.length - 1) sb.append(", ");
        }
        sb.append("]");
        return sb.toString();
    }
}