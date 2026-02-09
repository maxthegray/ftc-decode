package org.firstinspires.ftc.teamcode.threaded.Old;

/**
 * Utility class for ball color classification and shoot-plan calculation.
 *
 * No state — just the BallColor enum and static helpers used by
 * MechanismThread, SensorState, BallClassifier, CameraThread, etc.
 */
public class ShootSequence {

    public enum BallColor { GREEN, PURPLE, EMPTY, UNKNOWN }

    private ShootSequence() {} // Not instantiable

    /**
     * Find how many slots to rotate so that 'color' ends up at the intake position.
     *
     * @return 0 if already at intake, +1 if at back-left (rotate right),
     *         -1 if at back-right (rotate left), or Integer.MIN_VALUE if not found.
     */
    public static int findRotationForColor(BallColor[] positions, BallColor color) {
        if (positions[0] == color) return 0;
        if (positions[1] == color) return 1;   // Back-left → rotate right
        if (positions[2] == color) return -1;   // Back-right → rotate left
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
     * @return int[] of length targetOrder.length, or null if impossible.
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

            // Ball shot — intake now empty
            intake = BallColor.EMPTY;
        }

        return plan;
    }
}