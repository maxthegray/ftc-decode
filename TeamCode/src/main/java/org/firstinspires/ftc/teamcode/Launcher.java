package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.Arrays;

public class Launcher {

    private Integer[] targetSequence;       // shooting order
    private ArrayList<Integer> carouselBalls; // current balls in carousel, slots 0, 1, 2
    private int carouselPosition = 0;       // current angle of carousel






    public void initBurst(Integer[] targetSequence, Integer[] carouselBallsArray) {
        this.targetSequence = targetSequence;
        this.carouselBalls = new ArrayList<>(Arrays.asList(carouselBallsArray));

        if (!this.carouselBalls.isEmpty()) {
            getNextBall(targetSequence[0]);
            rotateTargetSequence();
        }
    }

    public void shootBall() {
        //flicker & motor


    }

    public void getNextBall(int targetColor) {
        boolean colorFound = false;
        int slotIndex = -1;

        for (int i = 0; i < carouselBalls.size(); i++) {
            if (carouselBalls.get(i).equals(targetColor)) {
                colorFound = true;
                slotIndex = i;
                int rotationAngle = slotIndex * 120; // 0->0°, 1->120°, 2->240°
                rotateCarouselTo(rotationAngle);

                // Remove the ball from the carousel after positioning
                carouselBalls.remove(i);
                break;
            }
        }

        if (!colorFound) {
            //do next best thing
        }
    }

    private void rotateCarouselTo(int degrees) {
        carouselPosition = degrees % 360;
        //servo smth
    }

    private void rotateTargetSequence() {
        Integer first = targetSequence[0];
        for (int i = 0; i < targetSequence.length - 1; i++) {
            targetSequence[i] = targetSequence[i + 1];
        }
        targetSequence[targetSequence.length - 1] = first;

        for (Integer color : targetSequence) {
            System.out.print(color + " ");
        }
    }
}
