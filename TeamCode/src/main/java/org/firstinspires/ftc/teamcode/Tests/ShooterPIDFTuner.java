package org.firstinspires.ftc.teamcode.threaded;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * ══════════════════════════════════════════════════════════════════════════════
 *  SHOOTER PIDF TUNER
 * ══════════════════════════════════════════════════════════════════════════════
 *
 *  Tune P, I, D, F for the shooter motor without re-deploying.
 *  Once happy, copy the final values into ShooterThread.java.
 *
 *  ── GAMEPAD 1 CONTROLS ───────────────────────────────────────────────────────
 *
 *  D-pad Up / Down      — Select which term to adjust (P → I → D → F)
 *  D-pad Right / Left   — Increase / decrease selected term by step
 *  Right Bumper         — Hold for 10× step (coarse adjust)
 *  Left Bumper          — Hold for 0.1× step (fine adjust)
 *
 *  A                    — Spin up to TARGET velocity
 *  B                    — Stop motor
 *  X                    — Quick-toggle between TARGET and 0 (tap to start/stop)
 *  Y                    — Run MAX VELOCITY TEST (full power, no PIDF, reads max TPS)
 *
 *  Right Trigger        — Increase target velocity by 10
 *  Left Trigger         — Decrease target velocity by 10
 *
 *  ── TUNING PROCEDURE ─────────────────────────────────────────────────────────
 *
 *  STEP 1 — Find your motor's max velocity:
 *    Press Y. The motor runs at full power with PIDF disabled.
 *    Read "Max TPS" from telemetry once it stabilizes.
 *    Press B to stop.
 *
 *  STEP 2 — Calculate starting F:
 *    F = 32767.0 / maxTPS
 *    Use D-pad to select F and dial it in.
 *
 *  STEP 3 — Test with F only (P=0, I=0, D=0):
 *    Press A to spin up. The motor should reach ~80-90% of target.
 *    If it overshoots: lower F. If it undershoots: raise F.
 *
 *  STEP 4 — Add P to close the remaining gap:
 *    Select P and raise from 0. Start around 50-100.
 *    Press A. It should now reach target precisely.
 *    If it oscillates: lower P.
 *
 *  STEP 5 — Add D to damp oscillation (if any):
 *    Usually small. Try 0.1, raise slowly.
 *
 *  STEP 6 — Add I only if there's consistent steady-state error:
 *    Keep it tiny. Try 0.1-1.0.
 *
 *  STEP 7 — Test spinup time:
 *    Press B to stop, wait for 0 RPM, press A.
 *    Watch "Spinup Time" — this is what you're optimizing.
 *    Try different target velocities with the triggers.
 *
 *  STEP 8 — Copy values into ShooterThread.java.
 *
 * ══════════════════════════════════════════════════════════════════════════════
 */
@TeleOp(name = "Shooter PIDF Tuner", group = "Tuning")
public class ShooterPIDFTuner extends LinearOpMode {

    private DcMotorEx motor;

    // ── Live-tunable coefficients ────────────────────────────────────────────
    private double kP = 350.0;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kF = 12.0;

    // ── Target velocity (deg/s) ──────────────────────────────────────────────
    private double targetVelocity = 150.0;
    private boolean motorRunning = false;

    // ── Which coefficient is selected ────────────────────────────────────────
    private enum SelectedTerm { P, I, D, F }
    private SelectedTerm selected = SelectedTerm.F;  // Start on F since that's what we're tuning

    // ── Step sizes per term ──────────────────────────────────────────────────
    private double getStep() {
        switch (selected) {
            case P: return 10.0;
            case I: return 0.1;
            case D: return 0.01;
            case F: return 0.5;
            default: return 1.0;
        }
    }

    // ── Spinup timing ────────────────────────────────────────────────────────
    private ElapsedTime spinupTimer = new ElapsedTime();
    private boolean measuringSpinup = false;
    private double lastSpinupMs = -1;
    private static final double VELOCITY_TOLERANCE = 10.0;  // deg/s

    // ── Max velocity test ────────────────────────────────────────────────────
    private boolean maxVelocityTest = false;
    private double maxVelocityObserved = 0;

    // ── Edge detection ───────────────────────────────────────────────────────
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;
    private boolean prevUp = false;
    private boolean prevDown = false;
    private boolean prevLeft = false;
    private boolean prevRight = false;
    private boolean prevRT = false;
    private boolean prevLT = false;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "launcher_motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        applyPIDF();

        telemetry.addLine("Shooter PIDF Tuner Ready");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ── Term selection (D-pad Up/Down) ───────────────────────────
            if (gamepad1.dpad_up && !prevUp) {
                selected = prevTerm(selected);
            }
            prevUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !prevDown) {
                selected = nextTerm(selected);
            }
            prevDown = gamepad1.dpad_down;

            // ── Value adjustment (D-pad Left/Right) ──────────────────────
            double step = getStep();
            if (gamepad1.right_bumper) step *= 10.0;
            if (gamepad1.left_bumper) step *= 0.1;

            if (gamepad1.dpad_right && !prevRight) {
                adjustSelected(step);
                applyPIDF();
            }
            prevRight = gamepad1.dpad_right;

            if (gamepad1.dpad_left && !prevLeft) {
                adjustSelected(-step);
                applyPIDF();
            }
            prevLeft = gamepad1.dpad_left;

            // ── Target velocity (triggers) ───────────────────────────────
            boolean rtPressed = gamepad1.right_trigger > 0.1;
            if (rtPressed && !prevRT) {
                targetVelocity += 10;
            }
            prevRT = rtPressed;

            boolean ltPressed = gamepad1.left_trigger > 0.1;
            if (ltPressed && !prevLT) {
                targetVelocity = Math.max(0, targetVelocity - 10);
            }
            prevLT = ltPressed;

            // ── A: Spin up ───────────────────────────────────────────────
            if (gamepad1.a && !prevA) {
                if (maxVelocityTest) {
                    maxVelocityTest = false;
                    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    applyPIDF();
                }
                motorRunning = true;
                measuringSpinup = true;
                spinupTimer.reset();
                lastSpinupMs = -1;
            }
            prevA = gamepad1.a;

            // ── B: Stop ──────────────────────────────────────────────────
            if (gamepad1.b && !prevB) {
                motorRunning = false;
                maxVelocityTest = false;
                measuringSpinup = false;
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                applyPIDF();
                motor.setVelocity(0);
            }
            prevB = gamepad1.b;

            // ── X: Toggle ────────────────────────────────────────────────
            if (gamepad1.x && !prevX) {
                if (maxVelocityTest) {
                    maxVelocityTest = false;
                    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    applyPIDF();
                }
                motorRunning = !motorRunning;
                if (motorRunning) {
                    measuringSpinup = true;
                    spinupTimer.reset();
                    lastSpinupMs = -1;
                } else {
                    measuringSpinup = false;
                    motor.setVelocity(0);
                }
            }
            prevX = gamepad1.x;

            // ── Y: Max velocity test ─────────────────────────────────────
            if (gamepad1.y && !prevY) {
                motorRunning = false;
                measuringSpinup = false;
                maxVelocityTest = true;
                maxVelocityObserved = 0;
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setPower(1.0);
            }
            prevY = gamepad1.y;

            // ── Motor command ────────────────────────────────────────────
            double currentVelocity;

            if (maxVelocityTest) {
                currentVelocity = motor.getVelocity(AngleUnit.DEGREES);
                if (currentVelocity > maxVelocityObserved) {
                    maxVelocityObserved = currentVelocity;
                }
            } else if (motorRunning) {
                motor.setVelocity(targetVelocity, AngleUnit.DEGREES);
                currentVelocity = motor.getVelocity(AngleUnit.DEGREES);

                // Measure spinup time
                if (measuringSpinup) {
                    double error = Math.abs(currentVelocity - targetVelocity);
                    if (error <= VELOCITY_TOLERANCE) {
                        lastSpinupMs = spinupTimer.milliseconds();
                        measuringSpinup = false;
                    }
                }
            } else {
                motor.setVelocity(0);
                currentVelocity = motor.getVelocity(AngleUnit.DEGREES);
            }

            // ── Telemetry ────────────────────────────────────────────────
            telemetry.addLine("══ SHOOTER PIDF TUNER ══════════════════");
            telemetry.addLine();

            // Show coefficients — highlight selected one
            telemetry.addData(selected == SelectedTerm.P ? "► P" : "  P", "%.3f", kP);
            telemetry.addData(selected == SelectedTerm.I ? "► I" : "  I", "%.4f", kI);
            telemetry.addData(selected == SelectedTerm.D ? "► D" : "  D", "%.4f", kD);
            telemetry.addData(selected == SelectedTerm.F ? "► F" : "  F", "%.3f", kF);
            telemetry.addData("  Step", "%.4f (RB=10×, LB=0.1×)", getStep());
            telemetry.addLine();

            // Velocity info
            telemetry.addData("Target", "%.1f deg/s", targetVelocity);
            telemetry.addData("Current", "%.1f deg/s", currentVelocity);
            telemetry.addData("Error", "%.1f deg/s", Math.abs(currentVelocity - targetVelocity));
            telemetry.addData("Motor", motorRunning ? "RUNNING" : "STOPPED");
            telemetry.addLine();

            // Spinup timing
            if (lastSpinupMs >= 0) {
                telemetry.addData("Spinup Time", "%.0f ms", lastSpinupMs);
            } else if (measuringSpinup) {
                telemetry.addData("Spinup Time", "measuring... (%.0f ms)", spinupTimer.milliseconds());
            } else {
                telemetry.addData("Spinup Time", "press A to measure");
            }

            // Max velocity test
            if (maxVelocityTest) {
                telemetry.addLine();
                telemetry.addLine("── MAX VELOCITY TEST ACTIVE ──");
                telemetry.addData("Current TPS", "%.1f deg/s", currentVelocity);
                telemetry.addData("Max Observed", "%.1f deg/s", maxVelocityObserved);
                telemetry.addData("Suggested F", "%.3f", maxVelocityObserved > 0 ? 32767.0 / maxVelocityObserved : 0);
                telemetry.addLine("Press B to stop");
            }

            telemetry.addLine();
            telemetry.addLine("── CONTROLS ──");
            telemetry.addLine("A=Spin up  B=Stop  X=Toggle  Y=Max test");
            telemetry.addLine("DUp/Dn=Select  DL/R=Adjust  Triggers=Target vel");

            telemetry.update();
        }

        motor.setVelocity(0);
        motor.setPower(0);
    }

    // ── Helpers ──────────────────────────────────────────────────────────────

    private void applyPIDF() {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(kP, kI, kD, kF));
    }

    private void adjustSelected(double delta) {
        switch (selected) {
            case P: kP = Math.max(0, kP + delta); break;
            case I: kI = Math.max(0, kI + delta); break;
            case D: kD = Math.max(0, kD + delta); break;
            case F: kF = Math.max(0, kF + delta); break;
        }
    }

    private SelectedTerm nextTerm(SelectedTerm t) {
        switch (t) {
            case P: return SelectedTerm.I;
            case I: return SelectedTerm.D;
            case D: return SelectedTerm.F;
            case F: return SelectedTerm.P;
            default: return SelectedTerm.P;
        }
    }

    private SelectedTerm prevTerm(SelectedTerm t) {
        switch (t) {
            case P: return SelectedTerm.F;
            case I: return SelectedTerm.P;
            case D: return SelectedTerm.I;
            case F: return SelectedTerm.D;
            default: return SelectedTerm.P;
        }
    }
}