package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.Arrays;

public class Launcher {

    public Integer[] LtargetSequence;       // shooting order
    public ArrayList<Integer> LcarouselBalls; // current balls in carousel, slots 0, 1, 2
    public int LcarouselPosition = 0;       // current angle of carousel

    public void initBurst(Integer[] initTargetSequence, Integer[] initCarouselBallsArray) {
        this.LtargetSequence = initTargetSequence;
        this.LcarouselBalls = new ArrayList<>(Arrays.asList(initCarouselBallsArray));

        while (!this.LcarouselBalls.isEmpty()) {
            getNextBall(initTargetSequence[0]);
            rotateTargetSequence();
            shootBall();

        }
    }
    public void shootBall() {
        //flicker & motor
    }

    public void getNextBall(int targetColor) {
        boolean colorFound = false;
        int slotIndex = -1;

        for (int i = 0; i < LcarouselBalls.size(); i++) {
            if (LcarouselBalls.get(i).equals(targetColor)) {
                colorFound = true;
                slotIndex = i;
                int rotationAngle = slotIndex * 120; // 0->0°, 1->120°, 2->240°
                rotateCarouselTo(rotationAngle);

                // Remove the ball from the carousel after positioning
                LcarouselBalls.remove(i);
                break;
            }
        }

        if (!colorFound) {
            //do next best thing
        }
    }

    private void rotateCarouselTo(int degrees) {
        LcarouselPosition = degrees % 360;
        //servo smth
    }

    private void rotateTargetSequence() {
        Integer first = LtargetSequence[0];
        for (int i = 0; i < LtargetSequence.length - 1; i++) {
            LtargetSequence[i] = LtargetSequence[i + 1];
        }
        LtargetSequence[LtargetSequence.length - 1] = first;

    }
}
