package org.firstinspires.ftc.teamcode;

public class Launcher {
    private Integer[] LtargetSequence;   // shooting order
    private Integer[] LcarouselBalls;    // current balls in carousel, slots 0, 1, 2
    private int LcarouselPosition;       // current angle of carousel

    public Launcher(Integer[] initTargetSequence, Integer[] initCarouselBallsArray) {
        this.LtargetSequence = initTargetSequence;
        this.LcarouselBalls = initCarouselBallsArray;
        this.LcarouselPosition = 0;
    }

    public void doBurst() {
        getNextBall();
        rotateSequence(LtargetSequence);
        shootBall();
    }

    public void shootBall() {
        // flicker & motor
    }

    public void getNextBall() {
        int targetColor = LtargetSequence[0];

        boolean colorFound = false;
        int slotIndex = -1;

        for (int i = 0; i < LcarouselBalls.length; i++) {
            if (LcarouselBalls[i].equals(targetColor)) {
                colorFound = true;
                slotIndex = i;
                int rotationAngle = slotIndex * 120; // 0->0°, 1->120°, 2->240°
                rotateCarouselTo(rotationAngle);

                // Remove the ball from the carousel after positioning
                LcarouselBalls[slotIndex] = 0; // Assuming 0 means empty
                break;
            }
        }

        if (!colorFound) {
            // do next best thing
        }
    }

    private void rotateCarouselTo(int degrees) {
        LcarouselPosition = degrees % 360;
        // servo smth
    }

    private void rotateSequence(Integer[] trgtsequence) {
        Integer first = trgtsequence[0];
        for (int i = 0; i < trgtsequence.length - 1; i++) {
            trgtsequence[i] = trgtsequence[i + 1];
        }
        trgtsequence[trgtsequence.length - 1] = first;
    }
}
