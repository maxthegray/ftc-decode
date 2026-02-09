package org.firstinspires.ftc.teamcode.threaded.Old;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Controls the indicator lights.
 * Call update() each loop - handles auto-timeout internally.
 */
public class LightsController {

    private final Servo light1, light2, light3;
    private final ElapsedTime timer = new ElapsedTime();

    private boolean active = false;
    private static final double DURATION_SECONDS = 3.0;

    // PWM values for colors
    private static final double OFF = 0.0;
    private static final double GREEN = 0.500;
    private static final double PURPLE = 0.722;

    public enum Color { GREEN, PURPLE, OFF }

    public LightsController(HardwareMap hardwareMap) {
        light1 = hardwareMap.get(Servo.class, "light1");
        light2 = hardwareMap.get(Servo.class, "light2");
        light3 = hardwareMap.get(Servo.class, "light3");

        light1.setPosition(OFF);
        light2.setPosition(OFF);
        light3.setPosition(OFF);
    }

    /**
     * Show colors for the 3 positions. Turns off automatically after DURATION_SECONDS.
     */
    public void show(Color c1, Color c2, Color c3) {
        light1.setPosition(toValue(c1));
        light2.setPosition(toValue(c2));
        light3.setPosition(toValue(c3));
        active = true;
        timer.reset();
    }

    /**
     * Call every loop to handle auto-timeout.
     */
    public void update() {
        if (active && timer.seconds() >= DURATION_SECONDS) {
            light1.setPosition(OFF);
            light2.setPosition(OFF);
            light3.setPosition(OFF);
            active = false;
        }
    }

    public boolean isActive() {
        return active;
    }

    private double toValue(Color c) {
        switch (c) {
            case GREEN: return GREEN;
            case PURPLE: return PURPLE;
            default: return OFF;
        }
    }
}