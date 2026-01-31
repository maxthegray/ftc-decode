package org.firstinspires.ftc.teamcode.threaded.New;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static java.lang.Math.abs;

//@Disabled
@TeleOp

public class BlueRedTele extends LinearOpMode {
    
//    private Alliance alliance;

    private final float DRIVE_THRESHOLD = 0.05f;  // do NOT set below 0.05 !!!
    private final float SCALE_DRIVE = 1.0f;
    private final float SCALE_TURN = 0.8f;
    //private float stickX, stickY;
    
    private float[] driveVector;
    private DriveVector drive;
    
    private XdriveTrain xDrive;
    private ExpansionHubI2C expansionHubI2C;
    private ControlHubI2C controlHubI2C;
    private BotState state;
    
    
    @Override
    public void runOpMode() {
        
        //alliance = Alliance.RED;
        
        driveVector = new float[3];

        //stickX = 0.0f;
        //stickY = 0.0f;
        state = new BotState();

        drive = new DriveVector(SCALE_DRIVE, SCALE_TURN);

        controlHubI2C = new ControlHubI2C(state, hardwareMap);
        expansionHubI2C = new ExpansionHubI2C(state, hardwareMap);

        xDrive = new XdriveTrain(0.0, DRIVE_THRESHOLD, hardwareMap, state);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        controlHubI2C.start();
        expansionHubI2C.start();
        xDrive.start();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //stickX = gamepad1.right_stick_x;
            //stickY = gamepad1.right_stick_y;
            
            // below is for Blue Alliance - DO NOT DELETE
            
            driveVector = drive.vector(-1.0f * gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.left_trigger, gamepad1.right_trigger);
            //driveVector = drive.vector(-1.0f * stickX, stickY, gamepad1.left_trigger, gamepad1.right_trigger);
            if (driveVector[0] < DRIVE_THRESHOLD) {
                driveVector[0] = 0.0f;
                driveVector[1] = 0.0f;
            }
            
            
            // below is for Red Alliance - DO NOT DELETE
            /*
            //driveVector = drive.vector(stickX, -1.0f * stickY, gamepad1.left_trigger, gamepad1.right_trigger);
            driveVector = drive.vector(gamepad1.right_stick_x, -1.0f * gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);
            if (driveVector[0] < DRIVE_THRESHOLD) {
                driveVector[0] = 0.0f;
                driveVector[1] = 0.0f;
            }
            */
            
            xDrive.updateVector(driveVector);
            
            telemetry.addData("stickX:    ", gamepad1.right_stick_x);
            telemetry.addData("stickY:    ", gamepad1.right_stick_y);
            telemetry.addData("triggerL:  ", gamepad1.left_trigger);
            telemetry.addData("triggerR:  ", gamepad1.right_trigger);
            telemetry.addLine();
            telemetry.addData("Magnitude: ", driveVector[0]);
            telemetry.addData("Angle:     ", driveVector[1]);
            telemetry.addData("Turn:      ", driveVector[2]);
            
            telemetry.addData("Status", "Running");
            telemetry.update();

        }  // end while

        state.endThreads();
        
    }  // end runOpMode
    
    /*
    private void driveVectorZero() {
        driveVector[0] = 0.0f;
        driveVector[1] = 0.0f;
        driveVector[2] = 0.0f;
    }
    */
    
    
    
    
}  // end class
