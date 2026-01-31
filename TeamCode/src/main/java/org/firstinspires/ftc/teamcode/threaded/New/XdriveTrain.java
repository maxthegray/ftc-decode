package org.firstinspires.ftc.teamcode.threaded.New;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static java.lang.Math.abs;
import static java.lang.Math.PI;
import static java.lang.Math.sin;
import static java.lang.Math.cos;

public class XdriveTrain extends Thread {
    
    private final double PI_4TH = PI/4.0;
    private final double PI_3RD = PI/3.0;
    private final double KP = 0.35;
    
    private volatile float magnitude, angle, turn, driveThreshold;

    private double x, y, f, direction, deltaDirection;
    
    private final float TURN_THRESHOLD = 0.05f;
    
    private HardwareMap hardwareMap;
    private BotState state;
    
    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
    
    public XdriveTrain(double direction, float threshold, HardwareMap hardwareMap, BotState state) {
        
        this.magnitude = 0.0f;
        this.angle = 0.0f;
        this.turn = 0.0f;
        
        this.direction = direction;
        deltaDirection = 0.0;
        f = 0.0;
        
        this.driveThreshold = threshold;
        
        this.hardwareMap = hardwareMap;
        this.state = state;
        
        initializeMotors();
        
    }  // end constructor
    
    
    public void run() {

        while (!state.shouldKillThreads()) {

            // need to force +/- PI
            direction += deltaDirection;

            x = (double) magnitude * cos(angle - direction);
            y = (double) magnitude * sin(angle - direction);

            if (magnitude < driveThreshold) {

                // case: robot is stopped
                if (abs(turn) < TURN_THRESHOLD) {
                    setPowerZero();
                    direction = state.getHeading();
                }

                // case: robot is turning only
                else {
                    // power the wheels to turn the robot
                    frontRightMotor.setPower(turn);
                    frontLeftMotor.setPower(turn);
                    backRightMotor.setPower(turn);
                    backLeftMotor.setPower(turn);
                    direction = state.getHeading();
                }

            } else {

                f = direction - state.getHeading();

                if (abs(f) < PI_4TH) {
                    f = KP * f / PI_4TH;
                } else {
                    f = KP;
                }

                // need to replace wit motor method(x,y,f) that clips power
                frontRightMotor.setPower(x + y + f);
                frontLeftMotor.setPower(-x + y + f);
                backRightMotor.setPower(x - y + f);
                backLeftMotor.setPower(-x - y + f);

            }  // end if-else structure

            try {
                Thread.sleep(8l);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt(); // Restore the interrupt flag
            }
        }


    }  // end threaded method
    
    
    public void updateVector(float[] vector) {
        
        this.magnitude = vector[0];
        this.angle = vector[1];
        this.turn = vector[2];
        
    }  // end method updateVector
    
    
    private void setPowerZero() {
        
        frontRightMotor.setPower(0.0);
        frontLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        
    }  // end method setPowerZero
    
    
    private void initializeMotors() {
        
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
    }  // end method initialize
    
    

    
}  // end class
