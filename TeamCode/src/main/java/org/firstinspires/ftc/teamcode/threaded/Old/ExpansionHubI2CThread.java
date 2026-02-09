package org.firstinspires.ftc.teamcode.threaded.Old;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Reads back-left and back-right color sensors (Expansion Hub I2C bus).
 */
public class ExpansionHubI2CThread extends Thread {

    private final     SensorState state;
    private final RevColorSensorV3 backLeftA;
    private final RevColorSensorV3 backLeftB;
    private final RevColorSensorV3 backRightA;
    private final RevColorSensorV3 backRightB;

    public ExpansionHubI2CThread(    SensorState state, HardwareMap hardwareMap) {
        this.state = state;
        this.setPriority(Thread.MIN_PRIORITY);

        backLeftA = hardwareMap.get(RevColorSensorV3.class, "BL_color");
        backLeftB = hardwareMap.get(RevColorSensorV3.class, "BL_upper");
        backRightA = hardwareMap.get(RevColorSensorV3.class, "BR_color");
        backRightB = hardwareMap.get(RevColorSensorV3.class, "BR_upper");
    }

    @Override
    public void run() {
        while (!state.shouldKillThreads()) {
            // Back-left
            if (backLeftA != null) {
                state.setSensorValuesA(    SensorState.POS_BACK_LEFT,
                        backLeftA.alpha(), backLeftA.blue(), backLeftA.green());
            }
            if (backLeftB != null) {
                state.setSensorValuesB(    SensorState.POS_BACK_LEFT,
                        backLeftB.alpha(), backLeftB.blue(), backLeftB.green());
            }
            state.setPositionColor(    SensorState.POS_BACK_LEFT,
                        BallClassifier.classifyPosition(state,     SensorState.POS_BACK_LEFT));

            // Back-right
            if (backRightA != null) {
                state.setSensorValuesA(    SensorState.POS_BACK_RIGHT,
                        backRightA.alpha(), backRightA.blue(), backRightA.green());
            }
            if (backRightB != null) {
                state.setSensorValuesB(    SensorState.POS_BACK_RIGHT,
                        backRightB.alpha(), backRightB.blue(), backRightB.green());
            }
            state.setPositionColor(    SensorState.POS_BACK_RIGHT,
                        BallClassifier.classifyPosition(state,     SensorState.POS_BACK_RIGHT));

            try {
                Thread.sleep(    SensorState.I2C_UPDATE_MS);
            } catch (InterruptedException e) {
                break;
            }
        }
    }
}