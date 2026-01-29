package org.firstinspires.ftc.teamcode.threaded.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Test autonomous for OTOS drive controller.
 * Drives in a simple pattern to verify PID tuning.
 *
 * Pattern:
 *   1. Forward 24 inches
 *   2. Strafe right 24 inches
 *   3. Turn 90° right
 *   4. Forward 24 inches
 *   5. Return to start
 */
@Autonomous(name = "OTOS Drive Test", group = "Test")
public class OtosDriveTest extends LinearOpMode {

    private org.firstinspires.ftc.teamcode.threaded.Auto.OtosDriveController drive;
    private ElapsedTime runtime = new ElapsedTime();

    // Timeout for each movement (safety)
    private static final double MOVE_TIMEOUT_SEC = 5.0;

    @Override
    public void runOpMode() {
        // Initialize drive controller
        drive = new org.firstinspires.ftc.teamcode.threaded.Auto.OtosDriveController(hardwareMap);

        // Set starting pose (0, 0, 0°)
        drive.setStartPose(0, 0, 0);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start Pose", "(0, 0) @ 0°");
        telemetry.addLine();
        telemetry.addLine("This test will:");
        telemetry.addLine("1. Drive forward 24\"");
        telemetry.addLine("2. Strafe right 24\"");
        telemetry.addLine("3. Turn right 90°");
        telemetry.addLine("4. Drive forward 24\"");
        telemetry.addLine("5. Return to (0,0)");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // ===================== MOVEMENT 1: Forward 24" =====================
        telemetry.addData("Step", "1/5 - Forward 24\"");
        telemetry.update();

        drive.driveTo(24, 0, 0);
        waitUntilAtTarget("Forward 24\"");

        // ===================== MOVEMENT 2: Strafe Right 24" =====================
        telemetry.addData("Step", "2/5 - Strafe right 24\"");
        telemetry.update();

        drive.driveTo(24, -24, 0);
        waitUntilAtTarget("Strafe right 24\"");

        // ===================== MOVEMENT 3: Turn Right 90° =====================
        telemetry.addData("Step", "3/5 - Turn right 90°");
        telemetry.update();

        drive.turnTo(-90);
        waitUntilAtTarget("Turn right 90°");

        // ===================== MOVEMENT 4: Forward 24" =====================
        telemetry.addData("Step", "4/5 - Forward 24\"");
        telemetry.update();

        drive.driveForward(24);
        waitUntilAtTarget("Forward 24\"");

        // ===================== MOVEMENT 5: Return to Start =====================
        telemetry.addData("Step", "5/5 - Return to start");
        telemetry.update();

        drive.driveTo(0, 0, 0);
        waitUntilAtTarget("Return to start");

        // ===================== DONE =====================
        drive.stop();

        telemetry.addData("Status", "COMPLETE!");
        telemetry.addData("Total Time", "%.1f sec", runtime.seconds());
        telemetry.addData("Final Pose", "(%.1f, %.1f) @ %.1f°",
                drive.getCurrentX(), drive.getCurrentY(), drive.getCurrentHeadingDegrees());
        telemetry.update();

        // Keep displaying until stopped
        while (opModeIsActive()) {
            sleep(100);
        }
    }

    /**
     * Wait until the drive controller reaches target or timeout.
     */
    private void waitUntilAtTarget(String moveName) {
        ElapsedTime moveTimer = new ElapsedTime();

        while (opModeIsActive() && !drive.isAtTarget() && moveTimer.seconds() < MOVE_TIMEOUT_SEC) {
            drive.update();

            // Telemetry
            telemetry.addData("Move", moveName);
            telemetry.addData("Runtime", "%.1f sec", runtime.seconds());
            telemetry.addLine();
            telemetry.addData("Current", "(%.1f, %.1f) @ %.1f°",
                    drive.getCurrentX(), drive.getCurrentY(), drive.getCurrentHeadingDegrees());
            telemetry.addData("Target", "(%.1f, %.1f) @ %.1f°",
                    drive.getTargetX(), drive.getTargetY(), drive.getTargetHeadingDegrees());
            telemetry.addLine();
            telemetry.addData("Error X", "%.2f in", drive.getXError());
            telemetry.addData("Error Y", "%.2f in", drive.getYError());
            telemetry.addData("Error H", "%.2f°", drive.getHeadingErrorDegrees());
            telemetry.addLine();
            telemetry.addData("In Tolerance", drive.isWithinTolerance() ? "YES" : "no");
            telemetry.addData("At Target", drive.isAtTarget() ? "YES" : "no");
            telemetry.update();
        }

        // Brief pause between movements
        drive.stop();
        sleep(200);
    }
}