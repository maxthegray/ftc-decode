package org.firstinspires.ftc.teamcode.threaded.Old;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ExpansionHubI2CThread extends Thread {

    private final BotState state;

    private final RevColorSensorV3 backLeftSensorA;
    private final RevColorSensorV3 backLeftSensorB;
    private final RevColorSensorV3 backRightSensorA;
    private final RevColorSensorV3 backRightSensorB;

    public ExpansionHubI2CThread(BotState state, HardwareMap hardwareMap) {
        this.state = state;
        this.setPriority(Thread.MIN_PRIORITY);

        backLeftSensorA = hardwareMap.get(RevColorSensorV3.class, "BL_color");
        backLeftSensorB = hardwareMap.get(RevColorSensorV3.class, "BL_upper");
        backRightSensorA = hardwareMap.get(RevColorSensorV3.class, "BR_color");
        backRightSensorB = hardwareMap.get(RevColorSensorV3.class, "BR_upper");
    }

    @Override
    public void run() {
        while (!state.shouldKillThreads()) {


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

            state.setPositionColor(BotState.POS_BACK_LEFT,
                    BallClassifier.classifyPosition(state, BotState.POS_BACK_LEFT));
            state.setPositionColor(BotState.POS_BACK_RIGHT,
                    BallClassifier.classifyPosition(state, BotState.POS_BACK_RIGHT));

            try {
                Thread.sleep(BotState.I2C_UPDATE_MS);
            } catch (InterruptedException e) {
                break;
            }
        }
    }
}