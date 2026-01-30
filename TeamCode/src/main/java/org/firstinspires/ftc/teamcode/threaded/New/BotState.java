package org.firstinspires.ftc.teamcode.threaded.New;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;


public class BotState {

    private volatile boolean killThreads = false;

    public static final int POS_INTAKE = 0;
    public static final int POS_BACK_LEFT = 1;
    public static final int POS_BACK_RIGHT = 2;
    public static final int NUM_POSITIONS = 3;

    private volatile int[] alphaA = new int[NUM_POSITIONS];
    private volatile int[] alphaB = new int[NUM_POSITIONS];
    private volatile int[] blueA = new int[NUM_POSITIONS];
    private volatile int[] blueB = new int[NUM_POSITIONS];
    private volatile int[] greenA = new int[NUM_POSITIONS];
    private volatile int[] greenB = new int[NUM_POSITIONS];

    private volatile SparkFunOTOS.Pose2D otosPosition = new SparkFunOTOS.Pose2D(0, 0, 0);

    public BotState() {
    }

    public void endThreads() {
        killThreads = true;
    }

    public boolean shouldKillThreads() {
        return killThreads;
    }

    public void setSensorValuesA(int position, int alpha, int blue, int green) {
        if (position >= 0 && position < NUM_POSITIONS) {
            alphaA[position] = alpha;
            blueA[position] = blue;
            greenA[position] = green;
        }
    }

    public void setSensorValuesB(int position, int alpha, int blue, int green) {
        if (position >= 0 && position < NUM_POSITIONS) {
            alphaB[position] = alpha;
            blueB[position] = blue;
            greenB[position] = green;
        }
    }

    public int getAlphaA(int position) {
        return (position >= 0 && position < NUM_POSITIONS) ? alphaA[position] : 0;
    }

    public int getAlphaB(int position) {
        return (position >= 0 && position < NUM_POSITIONS) ? alphaB[position] : 0;
    }

    public int getBlueA(int position) {
        return (position >= 0 && position < NUM_POSITIONS) ? blueA[position] : 0;
    }

    public int getBlueB(int position) {
        return (position >= 0 && position < NUM_POSITIONS) ? blueB[position] : 0;
    }

    public int getGreenA(int position) {
        return (position >= 0 && position < NUM_POSITIONS) ? greenA[position] : 0;
    }

    public int getGreenB(int position) {
        return (position >= 0 && position < NUM_POSITIONS) ? greenB[position] : 0;
    }

    public void setOtosPosition(SparkFunOTOS.Pose2D position) {
        this.otosPosition = position;
    }

    public SparkFunOTOS.Pose2D getOtosPosition() {
        return otosPosition;
    }
}