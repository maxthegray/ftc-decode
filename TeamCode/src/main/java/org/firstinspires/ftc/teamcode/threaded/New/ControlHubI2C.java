package org.firstinspires.ftc.teamcode.threaded.New;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class ControlHubI2C extends Thread {

    private final BotState state;

    private final SparkFunOTOS otos;
    private final RevColorSensorV3 backLeftSensorA;
    private final RevColorSensorV3 backLeftSensorB;

    private int loopCounter = 0;

    public ControlHubI2C(BotState state, HardwareMap hardwareMap) {
        this.state = state;
        this.setName("ControlHubI2C");
        this.setPriority(Thread.MIN_PRIORITY);

        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);
        otos.setOffset(new SparkFunOTOS.Pose2D(0, 0, -90));
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);
        otos.calibrateImu();
        otos.resetTracking();

        backLeftSensorA = hardwareMap.get(RevColorSensorV3.class, "BL_color");
        backLeftSensorB = hardwareMap.get(RevColorSensorV3.class, "BL_upper");
    }

    @Override
    public void run() {
        while (!state.shouldKillThreads()) {
            state.setOtosPosition(otos.getPosition());

            if (loopCounter == 0) {
                readSensor(backLeftSensorA, true);
            } else if (loopCounter == 10) {
                readSensor(backLeftSensorB, false);
            }

            loopCounter++;
            if (loopCounter >= 20) {
                loopCounter = 0;
            }

            sleep();
        }
    }

    private void readSensor(RevColorSensorV3 sensor, boolean isSensorA) {
        if (sensor == null) return;

        if (isSensorA) {
            state.setSensorValuesA(BotState.POS_BACK_LEFT, sensor.alpha(), sensor.blue(), sensor.green());
        } else {
            state.setSensorValuesB(BotState.POS_BACK_LEFT, sensor.alpha(), sensor.blue(), sensor.green());
        }
    }

    private void sleep() {
        try {
            Thread.sleep(4L);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}