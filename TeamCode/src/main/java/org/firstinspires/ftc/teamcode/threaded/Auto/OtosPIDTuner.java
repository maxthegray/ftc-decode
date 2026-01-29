package org.firstinspires.ftc.teamcode.threaded.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * PID Tuning OpMode for OTOS Drive Controller.
 *
 * Controls:
 *   Left Stick    = Manual drive (for repositioning)
 *   Right Stick X = Manual rotation
 *
 *   A = Run test: Forward 24"
 *   B = Run test: Strafe right 24"
 *   X = Run test: Turn right 90°
 *   Y = Run test: Return to origin
 *
 *   D-pad Up/Down    = Adjust LINEAR_P (±0.01)
 *   D-pad Left/Right = Adjust HEADING_P (±0.1)
 *
 *   Left Bumper  = Reset pose to (0,0,0)
 *   Right Bumper = Emergency stop
 *
 * Tune LINEAR_P first (forward/strafe), then HEADING_P (turns).
 * Start with P only, add D if oscillating, add I if not reaching target.
 */
@TeleOp(name = "OTOS PID Tuner", group = "Tuning")
public class OtosPIDTuner extends LinearOpMode {

    private org.firstinspires.ftc.teamcode.threaded.Auto.OtosDriveController drive;
    private ElapsedTime runtime = new ElapsedTime();

    // Test state
    private enum TestState { IDLE, RUNNING }
    private TestState testState = TestState.IDLE;
    private String currentTest = "";
    private ElapsedTime testTimer = new ElapsedTime();
    private static final double TEST_TIMEOUT = 5.0;

    // Button edge detection
    private boolean prevA, prevB, prevX, prevY;
    private boolean prevDpadUp, prevDpadDown, prevDpadLeft, prevDpadRight;
    private boolean prevLB, prevRB;

    @Override
    public void runOpMode() {
        // Initialize
        drive = new org.firstinspires.ftc.teamcode.threaded.Auto.OtosDriveController(hardwareMap);
        drive.setStartPose(0, 0, 0);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine();
        telemetry.addLine("A = Forward 24\"");
        telemetry.addLine("B = Strafe right 24\"");
        telemetry.addLine("X = Turn right 90°");
        telemetry.addLine("Y = Return to origin");
        telemetry.addLine();
        telemetry.addLine("D-pad U/D = LINEAR_P");
        telemetry.addLine("D-pad L/R = HEADING_P");
        telemetry.addLine("LB = Reset pose");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // ===================== HANDLE TESTS =====================
            if (testState == TestState.RUNNING) {
                drive.update();

                // Check if done or timeout
                if (drive.isAtTarget() || testTimer.seconds() > TEST_TIMEOUT) {
                    drive.stop();
                    testState = TestState.IDLE;
                }
            }

            // ===================== HANDLE INPUT =====================

            // Manual drive (only when not running a test)
            if (testState == TestState.IDLE) {
                double forward = -gamepad1.left_stick_y * 0.5;
                double strafe = gamepad1.left_stick_x * 0.5;
                double rotate = gamepad1.right_stick_x * 0.3;

                if (Math.abs(forward) > 0.05 || Math.abs(strafe) > 0.05 || Math.abs(rotate) > 0.05) {
                    // Manual control - bypass PID
                    manualDrive(forward, strafe, rotate);
                } else {
                    drive.stop();
                }
            }

            // A = Forward 24"
            if (gamepad1.a && !prevA && testState == TestState.IDLE) {
                startTest("Forward 24\"", drive.getCurrentX() + 24, drive.getCurrentY(),
                        drive.getCurrentHeadingDegrees());
            }
            prevA = gamepad1.a;

            // B = Strafe right 24"
            if (gamepad1.b && !prevB && testState == TestState.IDLE) {
                startTest("Strafe right 24\"", drive.getCurrentX(),
                        drive.getCurrentY() - 24, drive.getCurrentHeadingDegrees());
            }
            prevB = gamepad1.b;

            // X = Turn right 90°
            if (gamepad1.x && !prevX && testState == TestState.IDLE) {
                startTest("Turn right 90°", drive.getCurrentX(), drive.getCurrentY(),
                        drive.getCurrentHeadingDegrees() - 90);
            }
            prevX = gamepad1.x;

            // Y = Return to origin
            if (gamepad1.y && !prevY && testState == TestState.IDLE) {
                startTest("Return to origin", 0, 0, 0);
            }
            prevY = gamepad1.y;

            // D-pad Up = Increase LINEAR_P
            if (gamepad1.dpad_up && !prevDpadUp) {
                org.firstinspires.ftc.teamcode.threaded.Auto.OtosDriveController.LINEAR_P += 0.01;
            }
            prevDpadUp = gamepad1.dpad_up;

            // D-pad Down = Decrease LINEAR_P
            if (gamepad1.dpad_down && !prevDpadDown) {
                org.firstinspires.ftc.teamcode.threaded.Auto.OtosDriveController.LINEAR_P = Math.max(0, org.firstinspires.ftc.teamcode.threaded.Auto.OtosDriveController.LINEAR_P - 0.01);
            }
            prevDpadDown = gamepad1.dpad_down;

            // D-pad Right = Increase HEADING_P
            if (gamepad1.dpad_right && !prevDpadRight) {
                org.firstinspires.ftc.teamcode.threaded.Auto.OtosDriveController.HEADING_P += 0.1;
            }
            prevDpadRight = gamepad1.dpad_right;

            // D-pad Left = Decrease HEADING_P
            if (gamepad1.dpad_left && !prevDpadLeft) {
                org.firstinspires.ftc.teamcode.threaded.Auto.OtosDriveController.HEADING_P = Math.max(0, org.firstinspires.ftc.teamcode.threaded.Auto.OtosDriveController.HEADING_P - 0.1);
            }
            prevDpadLeft = gamepad1.dpad_left;

            // Left Bumper = Reset pose
            if (gamepad1.left_bumper && !prevLB) {
                drive.setStartPose(0, 0, 0);
            }
            prevLB = gamepad1.left_bumper;

            // Right Bumper = Emergency stop
            if (gamepad1.right_bumper && !prevRB) {
                drive.stop();
                testState = TestState.IDLE;
            }
            prevRB = gamepad1.right_bumper;

            // ===================== TELEMETRY =====================
            telemetry.addLine("=== PID VALUES ===");
            telemetry.addData("LINEAR_P", "%.3f  (D-pad U/D)", org.firstinspires.ftc.teamcode.threaded.Auto.OtosDriveController.LINEAR_P);
            telemetry.addData("LINEAR_I", "%.3f", org.firstinspires.ftc.teamcode.threaded.Auto.OtosDriveController.LINEAR_I);
            telemetry.addData("LINEAR_D", "%.3f", org.firstinspires.ftc.teamcode.threaded.Auto.OtosDriveController.LINEAR_D);
            telemetry.addLine();
            telemetry.addData("HEADING_P", "%.2f  (D-pad L/R)", org.firstinspires.ftc.teamcode.threaded.Auto.OtosDriveController.HEADING_P);
            telemetry.addData("HEADING_I", "%.3f", org.firstinspires.ftc.teamcode.threaded.Auto.OtosDriveController.HEADING_I);
            telemetry.addData("HEADING_D", "%.3f", org.firstinspires.ftc.teamcode.threaded.Auto.OtosDriveController.HEADING_D);

            telemetry.addLine();
            telemetry.addLine("=== CURRENT POSE ===");
            telemetry.addData("Position", "(%.1f, %.1f)", drive.getCurrentX(), drive.getCurrentY());
            telemetry.addData("Heading", "%.1f°", drive.getCurrentHeadingDegrees());

            telemetry.addLine();
            telemetry.addLine("=== TEST STATUS ===");
            if (testState == TestState.RUNNING) {
                telemetry.addData("Running", currentTest);
                telemetry.addData("Time", "%.1f / %.1f sec", testTimer.seconds(), TEST_TIMEOUT);
                telemetry.addData("Error X", "%.2f in", drive.getXError());
                telemetry.addData("Error Y", "%.2f in", drive.getYError());
                telemetry.addData("Error H", "%.2f°", drive.getHeadingErrorDegrees());
                telemetry.addData("At Target", drive.isAtTarget() ? "YES" : "no");
            } else {
                telemetry.addData("Status", "IDLE - Press A/B/X/Y to test");
            }

            telemetry.addLine();
            telemetry.addLine("LB=Reset pose  RB=Stop");

            telemetry.update();
        }
    }

    private void startTest(String name, double x, double y, double heading) {
        currentTest = name;
        drive.driveTo(x, y, heading);
        testState = TestState.RUNNING;
        testTimer.reset();
    }

    /**
     * Manual drive for repositioning (bypasses PID).
     */
    private void manualDrive(double forward, double strafe, double rotate) {
        drive.manualDrive(forward, strafe, rotate);
    }
}