package org.firstinspires.ftc.teamcode.threaded.New;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Simple Go To Point Test", group = "Test")
public class GoToPointTest extends LinearOpMode {

    private BotState state;
    private ControlHubI2C controlHubI2C;
    private XdriveTrain xDrive;
    private GoToPoint goToPoint;

    private final float DRIVE_THRESHOLD = 0.05f;

    @Override
    public void runOpMode() {

        // Initialize everything
        state = new BotState();
        controlHubI2C = new ControlHubI2C(state, hardwareMap);
        xDrive = new XdriveTrain(0.0, DRIVE_THRESHOLD, hardwareMap, state);
        goToPoint = new GoToPoint();

        // Configure go-to-point
        goToPoint.setDriveSpeed(0.4);         // 40% power - nice and slow for testing
        goToPoint.setDistanceTolerance(0.05); // 5cm tolerance

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Robot will drive 0.5m forward");
        telemetry.update();

        waitForStart();

        // Start the threads
        controlHubI2C.start();
        xDrive.start();

        // Give OTOS a moment to start reading
        sleep(100);

        // Set target: 0.5 meters in the +X direction (forward)
        // Adjust this based on your coordinate system!
        goToPoint.setTarget(0.5, 0.0);

        // Drive to the point
        while (opModeIsActive() && !goToPoint.isAtTarget()) {

            // Get the drive vector and send to drivetrain
            float[] driveVector = goToPoint.update(state);
            xDrive.updateVector(driveVector);

            // Show what's happening
            telemetry.addData("Target", "X: 0.5m, Y: 0.0m");
            telemetry.addLine();
            telemetry.addData("Current X", "%.3f m", state.getOtosPosition().x);
            telemetry.addData("Current Y", "%.3f m", state.getOtosPosition().y);
            telemetry.addData("Heading", "%.2f rad", state.getOtosPosition().h);
            telemetry.addLine();
            telemetry.addData("Distance", "%.3f m", goToPoint.getDistanceToTarget(state));
            telemetry.addData("Drive Mag", "%.2f", driveVector[0]);
            telemetry.addData("Drive Angle", "%.2f rad", driveVector[1]);
            telemetry.addData("At Target?", goToPoint.isAtTarget());
            telemetry.update();
        }

        // Stop the robot
        xDrive.updateVector(new float[]{0, 0, 0});

        telemetry.addData("Status", "DONE! Reached target.");
        telemetry.update();

        // Keep displaying for a moment so you can see the result
        sleep(2000);

        // Clean up
        state.endThreads();
    }
}