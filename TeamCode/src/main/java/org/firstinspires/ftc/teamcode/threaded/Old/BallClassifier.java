package org.firstinspires.ftc.teamcode.threaded.Old;

import org.firstinspires.ftc.teamcode.threaded.Old.BotState.BallColor;

public class BallClassifier {

    public static BallColor classifyPosition(BotState state, int position) {
        int thresholdA = state.getThresholdA(position);
        int thresholdB = state.getThresholdB(position);

        BallColor typeA = classifyBall(
                state.getAlphaA(position),
                state.getBlueA(position),
                state.getGreenA(position),
                thresholdA
        );

        BallColor typeB = classifyBall(
                state.getAlphaB(position),
                state.getBlueB(position),
                state.getGreenB(position),
                thresholdB
        );

        return mergeReadings(typeA, typeB);
    }

    public static BallColor classifyBall(int alpha, int blue, int green, int threshold) {
        if (alpha < threshold) {
            return BallColor.EMPTY;
        }

        if (green == 0) {
            return (blue > 0) ? BallColor.PURPLE : BallColor.UNKNOWN;
        }

        double ratio = (double) blue / green;
        return (ratio > 1.0) ? BallColor.PURPLE : BallColor.GREEN;
    }

    public static BallColor mergeReadings(BallColor typeA, BallColor typeB) {
        if (typeA == typeB) {
            return typeA;
        }

        if (typeA == BallColor.UNKNOWN) return typeB;
        if (typeB == BallColor.UNKNOWN) return typeA;

        if (typeA == BallColor.EMPTY) return typeB;
        if (typeB == BallColor.EMPTY) return typeA;

        return typeA;
    }
}