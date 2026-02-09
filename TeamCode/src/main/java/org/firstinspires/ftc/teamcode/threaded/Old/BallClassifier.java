package org.firstinspires.ftc.teamcode.threaded.Old;

/**
 * Classifies ball color from sensor readings.
 */
public class BallClassifier {

    public static ShootSequence.BallColor classifyPosition(       SensorState state, int position) {
        int thresholdA = state.getThresholdA(position);
        int thresholdB = state.getThresholdB(position);

               ShootSequence.BallColor typeA = classifyBall(
                state.getAlphaA(position),
                state.getBlueA(position),
                state.getGreenA(position),
                thresholdA
        );

               ShootSequence.BallColor typeB = classifyBall(
                state.getAlphaB(position),
                state.getBlueB(position),
                state.getGreenB(position),
                thresholdB
        );

        return mergeReadings(typeA, typeB);
    }

    public static        ShootSequence.BallColor classifyBall(int alpha, int blue, int green, int threshold) {
        if (alpha < threshold) {
            return        ShootSequence.BallColor.EMPTY;
        }

        if (green == 0) {
            return (blue > 0) ?        ShootSequence.BallColor.PURPLE :        ShootSequence.BallColor.UNKNOWN;
        }

        double ratio = (double) blue / green;
        return (ratio > 1.0) ?        ShootSequence.BallColor.PURPLE :        ShootSequence.BallColor.GREEN;
    }

    public static        ShootSequence.BallColor mergeReadings(       ShootSequence.BallColor a,        ShootSequence.BallColor b) {
        if (a == b) return a;

        if (a ==        ShootSequence.BallColor.UNKNOWN) return b;
        if (b ==        ShootSequence.BallColor.UNKNOWN) return a;

        if (a ==        ShootSequence.BallColor.EMPTY) return b;
        if (b ==        ShootSequence.BallColor.EMPTY) return a;

        return a;  // If conflicting, prefer A
    }
}