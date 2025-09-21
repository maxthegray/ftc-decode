package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

public class Launcher {

    private List<Integer> LtargetSequence;   // shooting order
    private List<Integer> LcarouselBalls;    // current balls in carousel, slots 0, 1, 2
    public double LcarouselPosition;         // current angle of carousel

    public Servo servo;



    // Constructor
    public Launcher(List<Integer> initTargetSequence, List<Integer> initCarouselBalls, Servo servo) {
        this.LtargetSequence = new ArrayList<>(initTargetSequence); // copy input
        this.LcarouselBalls = new ArrayList<>(initCarouselBalls);   // copy input
        this.LcarouselPosition = 0;
        this.servo = servo;
        servo.setPosition(0);
    }


    // Convert string like "PPG" into LtargetSequence we need to hook up fast capture camera to do this
    public void stringToList(String input) {
        LtargetSequence = new ArrayList<>();
        for (char c : input.toCharArray()) {
            if (c == 'P') {
                LtargetSequence.add(1);
            } else if (c == 'G') {
                LtargetSequence.add(2);
            }
        }
    }

    // Perform one burst
    public void doBurst() {
        getNextBall();
        rotateSequence();
        shootBall();
    }

    // Placeholder for shooting logic
    public void shootBall() {
        // flicker & motor control here
    }

    // Move carousel to the next target ball
    public void getNextBall() {
        if (LtargetSequence.isEmpty()) return; // no targets
        int targetColor = LtargetSequence.get(0);

        boolean colorFound = false;
        for (int i = 0; i < LcarouselBalls.size(); i++) {
            if (LcarouselBalls.get(i) != null && LcarouselBalls.get(i) == targetColor) {
                colorFound = true;
                double rotationAngle = i * 120; // 0->0°, 1->120°, 2->240°
                rotateCarouselTo(rotationAngle);

                // Remove the ball from carousel
                LcarouselBalls.set(i, 0); // 0 means empty
                break;
            }
        }

        if (!colorFound) {
            // Optional: handle case when target color is missing
        }
    }

    // Rotate carousel servo
    private void rotateCarouselTo(double degrees) {
        LcarouselPosition = degrees / 360.0; // normalized 0-1 for servo
        servo.setPosition(LcarouselPosition);
    }

    // Rotate target sequence (shift first element to end)
    private void rotateSequence() {
        if (!LtargetSequence.isEmpty()) {
            int first = LtargetSequence.remove(0);
            LtargetSequence.add(first);
        }
    }

    // Optional: get the current target sequence for debugging
    public List<Integer> getTargetSequence() {
        return new ArrayList<>(LtargetSequence);
    }

    // Optional: get current carousel state for debugging
    public List<Integer> getCarouselBalls() {
        return new ArrayList<>(LcarouselBalls);
    }
}
