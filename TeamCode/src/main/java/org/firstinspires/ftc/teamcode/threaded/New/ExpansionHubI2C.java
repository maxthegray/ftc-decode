package org.firstinspires.ftc.teamcode.threaded.New;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ExpansionHubI2C extends Thread {

    private final BotState state;

    private final RevColorSensorV3 intakeSensorA;
    private final RevColorSensorV3 intakeSensorB;
    private final RevColorSensorV3 backRightSensorA;
    private final RevColorSensorV3 backRightSensorB;

    private int loopCounter = 0;

    public ExpansionHubI2C(BotState state, HardwareMap hardwareMap) {
        this.state = state;
        this.setName("ExpansionHubI2C");
        this.setPriority(Thread.MIN_PRIORITY);

        intakeSensorA = hardwareMap.get(RevColorSensorV3.class, "intake_color1");
        intakeSensorB = hardwareMap.get(RevColorSensorV3.class, "intake_color2");
        backRightSensorB = hardwareMap.get(RevColorSensorV3.class, "BR_upper");
        backRightSensorA = hardwareMap.get(RevColorSensorV3.class, "BR_color");
    }

    @Override
    public void run() {
        while (!state.shouldKillThreads()) {


            switch (loopCounter) {
                case 0:
                    readSensor(intakeSensorA, BotState.POS_INTAKE, true);
                    break;
                case 1:
                    readSensor(intakeSensorB, BotState.POS_INTAKE, false);
                    break;
                case 2:
                    readSensor(backRightSensorB, BotState.POS_BACK_RIGHT, false);
                    break;
                case 3:
                    readSensor(backRightSensorA, BotState.POS_BACK_RIGHT, true);
                    break;
            }

            loopCounter++;

            loopCounter = (loopCounter + 1) % 4;

            sleep();
        }
    }

    private void readSensor(RevColorSensorV3 sensor, int position, boolean isSensorA) {
        if (sensor == null) return;

        if (isSensorA) {
            state.setSensorValuesA(position, sensor.alpha(), sensor.blue(), sensor.green());
        } else {
            state.setSensorValuesB(position, sensor.alpha(), sensor.blue(), sensor.green());
        }
    }

    private void sleep() {
        try {
            Thread.sleep(20L);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}