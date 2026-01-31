package org.firstinspires.ftc.teamcode.threaded.New;

import static com.qualcomm.robotcore.util.Range.clip;

import static java.lang.Math.sqrt;
import static java.lang.Math.atan2;

public class DriveVector {
    
    private float[] vector;
    // [0] is magnitude
    // [1] is angle in radians
    // [2] is turn power
    
    private float scaleTurn, scaleMag;
    
    
    public DriveVector(float scaleMag, float scaleTurn) {
        this.vector = new float[3];
        this.scaleTurn = scaleTurn;
        this.scaleMag = scaleMag;
    }  // end constructor
    
    
    public float[] vector(float stickX, float stickY, float triggerL, float triggerR) {
        
        vector[0] = scaleMag * (float) clip(sqrt(stickX * stickX + stickY * stickY), 0.0, 1.0);
        
        vector[1] = (float) (atan2(stickY, stickX));
        
        vector[2] = scaleTurn * (triggerL - triggerR);
        
        return vector;
        
    }  // end method make

}  // end class