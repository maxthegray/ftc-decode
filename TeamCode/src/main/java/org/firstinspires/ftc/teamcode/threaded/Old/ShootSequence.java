package org.firstinspires.ftc.teamcode.threaded.Old;

/**
 * Utility class for ball color classification and shoot-plan calculation.
 *
 * No state — just the BallColor enum and static helpers used by
 * MechanismThread, SensorState, BallClassifier, CameraThread, etc.
 *
 * When a needed color can't be found, methods fall back to kicking an EMPTY
 * slot so that the full shoot sequence always completes (even with missing balls).
 */
public class ShootSequence {

    public enum BallColor { GREEN, PURPLE, EMPTY, UNKNOWN }

    private ShootSequence() {} // Not instantiable

    /**
     * Find how many slots to rotate so that 'color' ends up at the intake position.
     *
     * Falls back to an EMPTY slot if the requested color isn't present,
     * so the sequence can still fire (kicking nothing on that turn).
     *
     * @return 0 if already at intake, +1 if at back-left (rotate right),
     *         -1 if at back-right (rotate left), or Integer.MIN_VALUE if
     *         no matching slot AND no empty slot exist (shouldn't happen).
     */
    public static int findRotationForColor(BallColor[] positions, BallColor color) {
        // First pass: look for the exact color
        if (positions[0] == color) return 0;
        if (positions[1] == color) return 1;   // Back-left → rotate right
        if (positions[2] == color) return -1;   // Back-right → rotate left

        // Fallback: kick an empty slot (fire nothing) to keep the sequence moving
        if (positions[0] == BallColor.EMPTY) return 0;
        if (positions[1] == BallColor.EMPTY) return 1;
        if (positions[2] == BallColor.EMPTY) return -1;

        return Integer.MIN_VALUE;
    }

    /**
     * Build a rotation plan to shoot balls in the given target order.
     *
     * Each entry is the rotation needed before that shot:
     *   0  = ball already at intake
     *   1  = rotate right (back-left → intake)
     *  -1  = rotate left  (back-right → intake)
     *
     * If a needed color is missing, the plan substitutes an EMPTY slot
     * (the kicker fires on nothing) so the full sequence always completes.
     *
     * @return int[] of length targetOrder.length, or null only if positions array is invalid.
     */
    public static int[] calculatePlan(BallColor[] positions, BallColor[] targetOrder) {
        if (positions == null || positions.length != 3) return null;
        if (targetOrder == null || targetOrder.length < 1 || targetOrder.length > 3) return null;

        // Work with copies
        BallColor intake    = positions[0];
        BallColor backLeft  = positions[1];
        BallColor backRight = positions[2];

        int[] plan = new int[targetOrder.length];

        for (int shot = 0; shot < targetOrder.length; shot++) {
            BallColor need = targetOrder[shot];

            // Try to find the needed color
            int rotation = findSlot(intake, backLeft, backRight, need);

            // Fallback: kick an empty slot instead
            if (rotation == Integer.MIN_VALUE) {
                rotation = findSlot(intake, backLeft, backRight, BallColor.EMPTY);
            }

            // Last resort: kick whatever is at intake (shouldn't normally happen)
            if (rotation == Integer.MIN_VALUE) {
                rotation = 0;
            }

            plan[shot] = rotation;

            // Simulate the rotation
            if (rotation == 1) {
                // Rotate right: back-left → intake
                BallColor temp = intake;
                intake = backLeft;
                backLeft = backRight;
                backRight = temp;
            } else if (rotation == -1) {
                // Rotate left: back-right → intake
                BallColor temp = intake;
                intake = backRight;
                backRight = backLeft;
                backLeft = temp;
            }

            // Ball shot — intake now empty
            intake = BallColor.EMPTY;
        }

        return plan;
    }

    /**
     * Find which slot holds the target color.
     *
     * @return 0 (intake), 1 (back-left), -1 (back-right), or Integer.MIN_VALUE if not found.
     */
    private static int findSlot(BallColor intake, BallColor backLeft, BallColor backRight, BallColor target) {
        if (intake == target) return 0;
        if (backLeft == target) return 1;
        if (backRight == target) return -1;
        return Integer.MIN_VALUE;
    }
}