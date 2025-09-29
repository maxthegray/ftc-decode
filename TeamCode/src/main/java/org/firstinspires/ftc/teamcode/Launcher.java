package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Launcher {

    private List<Integer> targetSequence;   // shooting order
    private List<Integer> ballsInCarousel;    // current balls in carousel, slots 0, 1, 2
    public double carouselPosition;         // current angle of carousel

    public Servo servo;
    boolean done = false;

    Telemetry telemetry;

    UnifiedLocalization Camera;

    // Constructor
    public Launcher(List<Integer> initTargetSequence, List<Integer> initCarouselBalls, HardwareMap hardwareMap, Telemetry telemetryy) {
        this.targetSequence = new ArrayList<>(initTargetSequence); // copy input
        this.ballsInCarousel = new ArrayList<>(initCarouselBalls);   // copy input
        this.carouselPosition = 0;
        this.servo = hardwareMap.get(Servo.class, "servo_sample");
        servo.setPosition(0);

        this.telemetry = telemetryy;

        Camera = new UnifiedLocalization(telemetryy, hardwareMap, 0,0,0);
    }

    public void step() {
        Camera.updateAprilTag();
        if (!done){
            if (Camera.colorID == 21) {
                targetSequence = Arrays.asList(2, 1, 1);
                done = true;
            } else if (Camera.colorID == 22) {
                targetSequence = Arrays.asList(1, 2, 1);
                done = true;
            } else if (Camera.colorID == 23) {
                targetSequence = Arrays.asList(1, 1, 2);
                done = true;
            }
        }

        addTelemetry(telemetry);

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
        if (targetSequence.isEmpty()) return; // no targets
        int targetColor = targetSequence.get(0);

        boolean colorFound = false;
        for (int i = 0; i < ballsInCarousel.size(); i++) {
            if (ballsInCarousel.get(i) != null && ballsInCarousel.get(i) == targetColor) {
                colorFound = true;
                double rotationAngle = i * 120; // 0->0°, 1->120°, 2->240°
                rotateCarouselTo(rotationAngle);

                // Remove the ball from carousel
                ballsInCarousel.set(i, 0); // 0 means empty
                break;
            }
        }

        if (!colorFound) {
            // Optional: handle case when target color is missing
        }
    }

    // Rotate carousel servo
    private void rotateCarouselTo(double degrees) {
        carouselPosition = degrees / 355.0; // normalized 0-1 for servo
        servo.setPosition(carouselPosition);
    }

    // Rotate target sequence (shift first element to end)
    private void rotateSequence() {
        if (!targetSequence.isEmpty()) {
            int first = targetSequence.remove(0);
            targetSequence.add(first);
        }
    }

    // Optional: get the current target sequence for debugging
    public List<Integer> getTargetSequence() {
        return new ArrayList<>(targetSequence);
    }

    // Optional: get current carousel state for debugging
    public List<Integer> getBallsInCarousel() {
        return new ArrayList<>(ballsInCarousel);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Target Sequence", targetSequence.toString());
        telemetry.addData("Carousel Balls", ballsInCarousel.toString());
        telemetry.addData("Carousel Position", carouselPosition);
        telemetry.addData("Visible Tags", Camera.canSeeTag() ? "Yes" : "No");
    }


}
