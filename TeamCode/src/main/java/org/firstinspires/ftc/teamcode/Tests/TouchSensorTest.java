package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
@Disabled
@TeleOp(name = "hi", group = "Test")
public class TouchSensorTest extends OpMode {

    AnalogInput touchSensor;
    AnalogInput touchSensor2;
    static final double THRESHOLD_VOLTS = 1.65;

    @Override
    public void init() {
        touchSensor = hardwareMap.get(AnalogInput.class, "touch1");
        touchSensor2 = hardwareMap.get(AnalogInput.class, "touch2");

        telemetry.update();
    }

    @Override
    public void loop() {
        double voltage = touchSensor.getVoltage();
        double voltage2 = touchSensor2.getVoltage();
        double maxVoltage = touchSensor.getMaxVoltage();
        boolean triggered = voltage >= THRESHOLD_VOLTS;

        telemetry.addData("Voltage1", "%.3f V", voltage);
        telemetry.addData("Voltage2", "%.3f V", voltage2);
        telemetry.addData("Threshold", "%.3f V", THRESHOLD_VOLTS);
        telemetry.addData("Triggered", triggered);
        telemetry.update();
    }
}