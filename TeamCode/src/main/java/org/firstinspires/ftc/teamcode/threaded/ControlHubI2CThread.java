package org.firstinspires.ftc.teamcode.threaded;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.threaded.BotState.BallColor;

/**
 * Thread for reading color sensors on Control Hub I2C bus.
 * Handles intake position sensors.
 */
public class ControlHubI2CThread extends Thread {

    private final BotState state;

    private final RevColorSensorV3 intakeSensorA;
    private final RevColorSensorV3 intakeSensorB;

    public ControlHubI2CThread(BotState state, HardwareMap hardwareMap) {
        this.state = state;
        this.setPriority(Thread.MIN_PRIORITY);  // Low priority - I2C is slow

        intakeSensorA = hardwareMap.get(RevColorSensorV3.class, "intake_color1");
        intakeSensorB = hardwareMap.get(RevColorSensorV3.class, "intake_color2");
    }

    @Override
    public void run() {
        while (!state.shouldKillThreads()) {

            // Read intake sensors
            if (intakeSensorA != null) {
                int alpha = intakeSensorA.alpha();
                int blue = intakeSensorA.blue();
                int green = intakeSensorA.green();
                state.setSensorValuesA(BotState.POS_INTAKE, alpha, blue, green);
            }

            if (intakeSensorB != null) {
                int alpha = intakeSensorB.alpha();
                int blue = intakeSensorB.blue();
                int green = intakeSensorB.green();
                state.setSensorValuesB(BotState.POS_INTAKE, alpha, blue, green);
            }

            // Classify ball at intake position
            BallColor color = classifyPosition(BotState.POS_INTAKE);
            state.setPositionColor(BotState.POS_INTAKE, color);

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