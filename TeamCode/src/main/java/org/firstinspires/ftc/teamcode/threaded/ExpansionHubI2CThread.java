package org.firstinspires.ftc.teamcode.threaded;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.threaded.BotState.BallColor;

public class ExpansionHubI2CThread extends Thread {

    private final BotState state;

    private RevColorSensorV3 intakeColor1;  // intake_color1 - sensor A for INTAKE
    private RevColorSensorV3 intakeColor2;  // intake_color2 - sensor B for INTAKE
    private RevColorSensorV3 brUpper;       // BR_upper - sensor B for BACK_RIGHT

    private static final long UPDATE_INTERVAL_MS = 1000;

    public ExpansionHubI2CThread(BotState state, HardwareMap hardwareMap) {
        this.state = state;

        intakeColor1 = hardwareMap.get(RevColorSensorV3.class, "intake_color1");
        intakeColor2 = hardwareMap.get(RevColorSensorV3.class, "intake_color2");
        brUpper = hardwareMap.get(RevColorSensorV3.class, "BR_upper");
    }

    @Override
    public void run() {
        while (!state.shouldKillThreads()) {

            // Read intake_color1 (sensor A for INTAKE)
            int intake1Alpha = intakeColor1.alpha();
            int intake1Blue = intakeColor1.blue();
            int intake1Green = intakeColor1.green();
            state.setSensorValuesA(BotState.POS_INTAKE, intake1Alpha, intake1Blue, intake1Green);

            // Read intake_color2 (sensor B for INTAKE)
            int intake2Alpha = intakeColor2.alpha();
            int intake2Blue = intakeColor2.blue();
            int intake2Green = intakeColor2.green();
            state.setSensorValuesB(BotState.POS_INTAKE, intake2Alpha, intake2Blue, intake2Green);

            // Read BR_upper (sensor B for BACK_RIGHT)
            int brUpperAlpha = brUpper.alpha();
            int brUpperBlue = brUpper.blue();
            int brUpperGreen = brUpper.green();
            state.setSensorValuesB(BotState.POS_BACK_RIGHT, brUpperAlpha, brUpperBlue, brUpperGreen);

            // Classify INTAKE (we have both sensors)
            BallColor intakeA = classifyBall(intake1Alpha, intake1Blue, intake1Green, BotState.THRESHOLD_INTAKE);
            BallColor intakeB = classifyBall(intake2Alpha, intake2Blue, intake2Green, BotState.THRESHOLD_INTAKE);
            state.setPositionColor(BotState.POS_INTAKE, combineSensors(intakeA, intakeB));

            // Note: BACK_RIGHT classification is done by ControlHubI2CThread
            // This thread only updates sensor B values for BACK_RIGHT

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
        return a;
    }
}