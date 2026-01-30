package org.firstinspires.ftc.teamcode.threaded.Old;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.threaded.Old.BotState.BallColor;

/**
 * Thread for reading color sensors on Expansion Hub I2C bus.
 * Handles back left and back right position sensors.
 */
public class ExpansionHubI2CThread extends Thread {

    private final BotState state;

    private final RevColorSensorV3 backLeftSensorA;
    private final RevColorSensorV3 backLeftSensorB;
    private final RevColorSensorV3 backRightSensorA;
    private final RevColorSensorV3 backRightSensorB;

    public ExpansionHubI2CThread(BotState state, HardwareMap hardwareMap) {
        this.state = state;
        this.setPriority(Thread.MIN_PRIORITY);  // Low priority - I2C is slow

        backLeftSensorA = hardwareMap.get(RevColorSensorV3.class, "BL_color");
        backLeftSensorB = hardwareMap.get(RevColorSensorV3.class, "BL_upper");
        backRightSensorA = hardwareMap.get(RevColorSensorV3.class, "BR_color");
        backRightSensorB = hardwareMap.get(RevColorSensorV3.class, "BR_upper");
    }

    @Override
    public void run() {
        while (!state.shouldKillThreads()) {

            // Read back left sensors
            if (backLeftSensorA != null) {
                int alpha = backLeftSensorA.alpha();
                int blue = backLeftSensorA.blue();
                int green = backLeftSensorA.green();
                state.setSensorValuesA(BotState.POS_BACK_LEFT, alpha, blue, green);
            }

            if (backLeftSensorB != null) {
                int alpha = backLeftSensorB.alpha();
                int blue = backLeftSensorB.blue();
                int green = backLeftSensorB.green();
                state.setSensorValuesB(BotState.POS_BACK_LEFT, alpha, blue, green);
            }

            // Read back right sensors
            if (backRightSensorA != null) {
                int alpha = backRightSensorA.alpha();
                int blue = backRightSensorA.blue();
                int green = backRightSensorA.green();
                state.setSensorValuesA(BotState.POS_BACK_RIGHT, alpha, blue, green);
            }

            if (backRightSensorB != null) {
                int alpha = backRightSensorB.alpha();
                int blue = backRightSensorB.blue();
                int green = backRightSensorB.green();
                state.setSensorValuesB(BotState.POS_BACK_RIGHT, alpha, blue, green);
            }

            // Classify balls at back positions
            state.setPositionColor(BotState.POS_BACK_LEFT, classifyPosition(BotState.POS_BACK_LEFT));
            state.setPositionColor(BotState.POS_BACK_RIGHT, classifyPosition(BotState.POS_BACK_RIGHT));

            try {
                Thread.sleep(BotState.I2C_UPDATE_MS);
            } catch (InterruptedException e) {
                break;
            }
        }
    }

    private BallColor classifyPosition(int position) {
        int thresholdA = state.getThresholdA(position);
        int thresholdB = state.getThresholdB(position);

        BallColor typeA = classifyBall(state.getAlphaA(position), state.getBlueA(position), state.getGreenA(position), thresholdA);
        BallColor typeB = classifyBall(state.getAlphaB(position), state.getBlueB(position), state.getGreenB(position), thresholdB);

        // Merge readings
        if (typeA == typeB) return typeA;
        if (typeA == BallColor.UNKNOWN) return typeB;
        if (typeB == BallColor.UNKNOWN) return typeA;
        if (typeA == BallColor.EMPTY) return typeB;
        if (typeB == BallColor.EMPTY) return typeA;
        return typeA;
    }

    private BallColor classifyBall(int alpha, int blue, int green, int threshold) {
        if (alpha < threshold) return BallColor.EMPTY;

        if (green == 0) {
            return (blue > 0) ? BallColor.PURPLE : BallColor.UNKNOWN;
        }

        double ratio = (double) blue / green;
        return (ratio > 1.0) ? BallColor.PURPLE : BallColor.GREEN;
    }
}