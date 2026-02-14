package org.firstinspires.ftc.teamcode.threaded.Old;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Reads intake position color sensors (Control Hub I2C bus).
 */
public class ControlHubI2CThread extends Thread {

    private final       SensorState state;
    private final RevColorSensorV3 sensorA;
    private final RevColorSensorV3 sensorB;

    public ControlHubI2CThread(      SensorState state, HardwareMap hardwareMap) {
        this.state = state;
        this.setPriority(Thread.MIN_PRIORITY);

        sensorA = hardwareMap.get(RevColorSensorV3.class, "intake_color1");
        sensorB = hardwareMap.get(RevColorSensorV3.class, "intake_color2");
    }

    @Override
    public void run() {
        while (!state.shouldKillThreads()) {
            if (!state.isCarouselSpinning()) {
                if (sensorA != null) {
                    state.setSensorValuesA(SensorState.POS_INTAKE,
                            sensorA.getDistance(DistanceUnit.MM), sensorA.blue(), sensorA.green());
                }
                if (sensorB != null) {
                    state.setSensorValuesB(SensorState.POS_INTAKE,
                            sensorB.getDistance(DistanceUnit.MM), sensorB.blue(), sensorB.green());
                }
                state.setPositionColor(SensorState.POS_INTAKE,
                        BallClassifier.classifyPosition(state, SensorState.POS_INTAKE));
            }

            try {
                Thread.sleep(SensorState.I2C_UPDATE_MS);
            } catch (InterruptedException e) {
                break;
            }
        }
    }
}