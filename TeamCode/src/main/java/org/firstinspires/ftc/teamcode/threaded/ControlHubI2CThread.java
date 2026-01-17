package org.firstinspires.ftc.teamcode.threaded;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.threaded.BotState.BallColor;

public class ControlHubI2CThread extends Thread {

    private final BotState state;

    private RevColorSensorV3 blColor;   // BL_color - sensor A for BACK_LEFT
    private RevColorSensorV3 blUpper;   // BL_upper - sensor B for BACK_LEFT
    private RevColorSensorV3 brColor;   // BR_color - sensor A for BACK_RIGHT

    private static final long UPDATE_INTERVAL_MS = 200;

    public ControlHubI2CThread(BotState state, HardwareMap hardwareMap) {
        this.state = state;

        blColor = hardwareMap.get(RevColorSensorV3.class, "BL_color");
        blUpper = hardwareMap.get(RevColorSensorV3.class, "BL_upper");
        brColor = hardwareMap.get(RevColorSensorV3.class, "BR_color");
    }

    @Override
    public void run() {
        while (!state.shouldKillThreads()) {

            // Read BL_color (sensor A for BACK_LEFT)
            int blColorAlpha = blColor.alpha();
            int blColorBlue = blColor.blue();
            int blColorGreen = blColor.green();
            state.setSensorValuesA(BotState.POS_BACK_LEFT, blColorAlpha, blColorBlue, blColorGreen);

            // Read BL_upper (sensor B for BACK_LEFT)
            int blUpperAlpha = blUpper.alpha();
            int blUpperBlue = blUpper.blue();
            int blUpperGreen = blUpper.green();
            state.setSensorValuesB(BotState.POS_BACK_LEFT, blUpperAlpha, blUpperBlue, blUpperGreen);

            // Read BR_color (sensor A for BACK_RIGHT)
            int brColorAlpha = brColor.alpha();
            int brColorBlue = brColor.blue();
            int brColorGreen = brColor.green();
            state.setSensorValuesA(BotState.POS_BACK_RIGHT, brColorAlpha, brColorBlue, brColorGreen);

            // Classify BACK_LEFT (we have both sensors)
            BallColor blA = classifyBall(blColorAlpha, blColorBlue, blColorGreen, BotState.THRESHOLD_BACK_LEFT);
            BallColor blB = classifyBall(blUpperAlpha, blUpperBlue, blUpperGreen, BotState.THRESHOLD_BACK_LEFT);
            state.setPositionColor(BotState.POS_BACK_LEFT, combineSensors(blA, blB));

            // Classify BACK_RIGHT (only sensor A, sensor B is on Expansion Hub)
            BallColor brA = classifyBall(brColorAlpha, brColorBlue, brColorGreen, BotState.THRESHOLD_BACK_RIGHT);
            BallColor brB = classifyBall(
                    state.getAlphaB(BotState.POS_BACK_RIGHT),
                    state.getBlueB(BotState.POS_BACK_RIGHT),
                    state.getGreenB(BotState.POS_BACK_RIGHT),
                    BotState.THRESHOLD_BACK_RIGHT
            );
            state.setPositionColor(BotState.POS_BACK_RIGHT, combineSensors(brA, brB));

            try {
                Thread.sleep(UPDATE_INTERVAL_MS);
            } catch (InterruptedException e) {
                break;
            }
        }
    }

    private BallColor classifyBall(int alpha, int blue, int green, int threshold) {
        if (alpha < threshold) {
            return BallColor.EMPTY;
        }

        if (green == 0) {
            return (blue > 0) ? BallColor.PURPLE : BallColor.UNKNOWN;
        }

        double ratio = (double) blue / green;
        return (ratio > 1.0) ? BallColor.PURPLE : BallColor.GREEN;
    }

    private BallColor combineSensors(BallColor a, BallColor b) {
        if (a == b) return a;
        if (a == BallColor.UNKNOWN) return b;
        if (b == BallColor.UNKNOWN) return a;
        if (a == BallColor.EMPTY) return b;
        if (b == BallColor.EMPTY) return a;
        return a; // Default to sensor A
    }
}