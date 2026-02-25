package org.firstinspires.ftc.teamcode.threaded.Old;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Controls the kicker servo and reads voltage feedback.
 * No timing logic - that's handled by the coordinator.
 */
public class KickerController {

    private final Servo servo;
    private final AnalogInput voltageFeedback;

    // Servo positions
    private static final double POS_DOWN = 0.0;
    private static final double POS_UP = 0.3;

    // Voltage threshold - kicker is "down" when voltage <= this
    public static final double DOWN_VOLTAGE_THRESHOLD = 1.3;

    public KickerController(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "flicker_servo");
        voltageFeedback = hardwareMap.get(AnalogInput.class, "flick");

        // Start in down position
        servo.setPosition(POS_DOWN);
    }

    /**
     * Command servo to up position.
     */
    public void up() {
        servo.setPosition(POS_UP);
    }

    /**
     * Command servo to down position.
     */
    public void down() {
        servo.setPosition(POS_DOWN);
    }

    /**
     * Is the kicker physically in the down position?
     * Based on voltage feedback, not commanded position.
     */
    public boolean isDown() {
        return getVoltage() <= DOWN_VOLTAGE_THRESHOLD;
    }

    /**
     * Raw voltage reading for telemetry.
     */
    public double getVoltage() {
        return voltageFeedback.getVoltage();
    }
}