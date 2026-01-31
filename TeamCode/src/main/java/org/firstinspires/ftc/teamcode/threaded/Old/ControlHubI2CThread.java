package org.firstinspires.ftc.teamcode.threaded.Old;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ControlHubI2CThread extends Thread {

    private final BotState state;

    private final RevColorSensorV3 intakeSensorA;
    private final RevColorSensorV3 intakeSensorB;

    public ControlHubI2CThread(BotState state, HardwareMap hardwareMap) {
        this.state = state;
        this.setPriority(Thread.MIN_PRIORITY);

        intakeSensorA = hardwareMap.get(RevColorSensorV3.class, "intake_color1");
        intakeSensorB = hardwareMap.get(RevColorSensorV3.class, "intake_color2");
    }

    @Override
    public void run() {
        while (!state.shouldKillThreads()) {

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

            state.setPositionColor(BotState.POS_INTAKE,
                    BallClassifier.classifyPosition(state, BotState.POS_INTAKE));

            try {
                Thread.sleep(BotState.I2C_UPDATE_MS);
            } catch (InterruptedException e) {
                break;
            }
        }
    }
}