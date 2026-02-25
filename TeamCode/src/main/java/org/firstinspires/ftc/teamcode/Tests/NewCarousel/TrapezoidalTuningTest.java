package org.firstinspires.ftc.teamcode.Tests.NewCarousel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/**
 * ═══════════════════════════════════════════════════════════════════════════
 *  TRAPEZOIDAL PROFILE TUNING TEST
 *  ─────────────────────────────────────────────────────────────────────────
 *  Use this FIRST to find kV, kP, maxVelocity, maxAcceleration.
 *  Then copy your final values into TimingCalibrationTest.
 *
 *  ─── THE CONTROLLER ──────────────────────────────────────────────────────
 *
 *  At every loop tick:
 *    1. Advance a trapezoidal velocity profile by dt seconds
 *       (ramp up at maxAcceleration, cruise at maxVelocity, ramp down to stop)
 *    2. Command: power = kV × profileVelocity + kP × (targetTicks − currentTicks)
 *
 *  The kV term (feedforward) does the bulk of the work — the motor is told
 *  exactly how much power to apply at every moment.
 *  The kP term (position correction) handles disturbances, overshoot, and
 *  holds position once settled. It stays calm because the profile keeps
 *  the position error small throughout the move.
 *
 *  ─── TUNING ORDER ────────────────────────────────────────────────────────
 *
 *  STEP 0 — Measure kV automatically (do this first, every session)
 *    Press X. The motor runs at 50% power for 1.5 seconds and measures its
 *    free-run velocity. kV is set to 0.5 / measured_velocity automatically.
 *    Run this 3 times and average, then lock in the value.
 *    Good kV → "vel_err" stays close to 0 during the cruise phase.
 *
 *  STEP 1 — Set maxVelocity (DPAD L/R, steps of 50 ticks/s)
 *    Start at the default and run moves with A. Increase until you see
 *    "vel_err" during cruise growing beyond ~50 ticks/s — the motor can't
 *    keep up anymore. Back off 10%. That's your maxVelocity.
 *    Log column: "vErr_max" — want this < 50 ticks/s during cruise.
 *
 *  STEP 2 — Set maxAcceleration (LB/RB, steps of 200 ticks/s²)
 *    Increase until "posErr_max" during acceleration starts growing (motor
 *    can't follow the profile ramp). Back off 15%.
 *    Log column: "pErr_max" — want this < 30 ticks during the move.
 *
 *  STEP 3 — Tune kP (L-trig/R-trig, steps of 0.00005)
 *    kP should be small. Increase until settle is tight without oscillation.
 *    Oscillation shows up as "overshoot=YES" in the log.
 *    Log column: "settle_ms" + "overshoot".
 *
 *  STEP 4 — Tune kV fine (DPAD D/U, steps of 0.000005)
 *    If "vErr_max" during cruise is consistently positive, increase kV slightly.
 *    If consistently negative, decrease. You want it centred around 0.
 *
 *  ─── CONTROLS ────────────────────────────────────────────────────────────
 *
 *    A            → Run a 1-slot move (alternates L/R)
 *    X            → Measure kV automatically (run at 50% power, read velocity)
 *    B            → View / page log
 *    START        → Clear log
 *
 *    DPAD L/R     → maxVelocity   − / +  (step 50 ticks/s)
 *    LB / RB      → maxAccel      − / +  (step 200 ticks/s²)
 *    DPAD D/U     → kV            − / +  (step 0.000005)
 *    L-trig/R-trig→ kP            − / +  (step 0.00005)  [pull > 0.5]
 *
 *  ─── LOG FORMAT ──────────────────────────────────────────────────────────
 *
 *    R## | vMax=XXXX | aMax=XXXXX | kV=0.000XXX | kP=0.000XX |
 *         pErr_max=XXX | vErr_max=XXX | settle_ms=XXX | overshoot=YES/NO
 *
 *    pErr_max  = worst position error during the move (want < 30 ticks)
 *    vErr_max  = worst velocity error during cruise   (want < 50 ticks/s)
 *    settle_ms = time from move start to within SETTLED_TOLERANCE ticks
 *    overshoot = did position error cross zero after approaching target?
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */
@TeleOp(name = "Trapezoidal Tuning Test", group = "Test")
public class TrapezoidalTuningTest extends LinearOpMode {

    // ══════════════════════════════════════════════════════
    //  HARDWARE
    // ══════════════════════════════════════════════════════

    private DcMotorEx motor;

    // ══════════════════════════════════════════════════════
    //  TUNABLE CONSTANTS
    // ══════════════════════════════════════════════════════

    // Starting values are conservative — the kV measurement mode in STEP 0
    // will replace kV automatically. Raise maxVelocity and maxAcceleration
    // until tracking starts to break down, then back off.
    private double maxVelocity     = 1000.0; // ticks/s  — raise in STEP 1
    private double maxAcceleration = 3000.0; // ticks/s² — raise in STEP 2
    private double kV              = 0.0007; // replaced by STEP 0 measurement
    private double kP              = 0.0003; // tune in STEP 3

    // Settle detection threshold — not tunable here, just for measurement
    private static final int SETTLED_TOLERANCE = 8; // ticks

    // ══════════════════════════════════════════════════════
    //  FIXED CONSTANTS
    // ══════════════════════════════════════════════════════

    private static final int    TICKS_PER_SLOT  = 2731;
    private static final double KV_MEASURE_POWER = 0.75; // power used for kV measurement
    private static final double KV_MEASURE_SECS  = 1.5; // how long to run for measurement

    // ══════════════════════════════════════════════════════
    //  STATE
    // ══════════════════════════════════════════════════════

    private enum TestState {
        IDLE,
        MOVING,         // Running a trapezoidal profile move
        KV_MEASURING,   // Running at fixed power to measure free velocity
        COMPLETE        // Move finished, waiting for next A press
    }
    private TestState testState = TestState.IDLE;

    // Profile runtime state
    private double profileVelocity  = 0;  // ticks/s — the profile's current commanded velocity
    private double profilePosition  = 0;  // ticks   — the profile's integrated position
    private int    moveTarget       = 0;  // encoder ticks — absolute target
    private int    moveStart        = 0;
    private int    rotateDir        = 1;  // alternates each move

    // Per-move measurements
    private long   moveStartMs      = 0;
    private long   settleMs         = 0;
    private int    peakPosError     = 0;  // max |posError| seen during move
    private double peakVelError     = 0;  // max |velError| seen during cruise phase
    private boolean overshot        = false;
    private boolean settled         = false;
    private int    lastPosError     = Integer.MAX_VALUE; // for overshoot detection

    // Velocity estimation (finite difference)
    private int    lastEncoderPos   = 0;
    private double estimatedVelocity = 0; // ticks/s

    // kV measurement
    private long   kvMeasureStartMs = 0;
    private double kvSumVelocity    = 0;
    private int    kvSampleCount    = 0;
    private double lastMeasuredKv   = -1;

    // ══════════════════════════════════════════════════════
    //  LOG
    // ══════════════════════════════════════════════════════

    private final List<String> logLines = new ArrayList<>();
    private int runNumber  = 0;
    private boolean showingLog = false;
    private int     logPage    = 0;
    private static final int LINES_PER_PAGE = 8;

    // ══════════════════════════════════════════════════════
    //  TIMERS
    // ══════════════════════════════════════════════════════

    private final ElapsedTime loopTimer  = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();

    // ══════════════════════════════════════════════════════
    //  EDGE DETECTION
    // ══════════════════════════════════════════════════════

    private boolean prevA  = false, prevX  = false;
    private boolean prevB  = false, prevSt = false;
    private boolean prevDL = false, prevDR = false;
    private boolean prevDU = false, prevDD = false;
    private boolean prevLB = false, prevRB = false;
    private boolean prevLT = false, prevRT = false;

    // ══════════════════════════════════════════════════════
    //  ENTRY POINT
    // ══════════════════════════════════════════════════════

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "carousel_motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // needed for getVelocity if wanted later
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // we handle control ourselves
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("TRAPEZOIDAL TUNING — Ready.  START on field controller.");
        telemetry.addLine("Press X first to auto-measure kV.");
        telemetry.update();
        waitForStart();

        lastEncoderPos = motor.getCurrentPosition();
        loopTimer.reset();

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();
            // Clamp dt — first loop and any hiccup should not blow up the profile
            dt = Math.min(dt, 0.05);

            handleInput();
            runController(dt);
            showTelemetry();
            sleep(10);
        }

        motor.setPower(0);
    }

    // ══════════════════════════════════════════════════════
    //  INPUT
    // ══════════════════════════════════════════════════════

    private void handleInput() {
        boolean curA  = gamepad1.a;
        boolean curX  = gamepad1.x;
        boolean curB  = gamepad1.b;
        boolean curSt = gamepad1.start;
        boolean curDL = gamepad1.dpad_left;
        boolean curDR = gamepad1.dpad_right;
        boolean curDU = gamepad1.dpad_up;
        boolean curDD = gamepad1.dpad_down;
        boolean curLB = gamepad1.left_bumper;
        boolean curRB = gamepad1.right_bumper;
        boolean curLT = gamepad1.left_trigger  > 0.5f;
        boolean curRT = gamepad1.right_trigger > 0.5f;

        if (curB && !prevB) {
            if (!showingLog) {
                showingLog = true; logPage = 0;
            } else {
                int pages = (int) Math.ceil((double) logLines.size() / LINES_PER_PAGE);
                if (++logPage >= Math.max(1, pages)) { showingLog = false; logPage = 0; }
            }
        }

        if (curSt && !prevSt) {
            logLines.clear(); runNumber = 0; showingLog = false;
        }

        boolean canAct = (testState == TestState.IDLE || testState == TestState.COMPLETE)
                && !showingLog;

        if (canAct) {
            if (curA && !prevA) startMove();
            if (curX && !prevX) startKvMeasure();

            // maxVelocity — DPAD L/R (step 50)
            if (curDL && !prevDL) maxVelocity = Math.max(100, maxVelocity - 50);
            if (curDR && !prevDR) maxVelocity = maxVelocity + 50;

            // maxAcceleration — LB/RB (step 200)
            if (curLB && !prevLB) maxAcceleration = Math.max(500, maxAcceleration - 200);
            if (curRB && !prevRB) maxAcceleration = maxAcceleration + 200;

            // kV — DPAD D/U (step 0.000005)
            if (curDD && !prevDD) kV = Math.max(0, kV - 0.000005);
            if (curDU && !prevDU) kV = kV + 0.000005;

            // kP — L-trig/R-trig (step 0.00005)
            if (curLT && !prevLT) kP = Math.max(0, kP - 0.00005);
            if (curRT && !prevRT) kP = kP + 0.00005;
        }

        prevA  = curA;  prevX  = curX;  prevB  = curB;  prevSt = curSt;
        prevDL = curDL; prevDR = curDR; prevDU = curDU; prevDD = curDD;
        prevLB = curLB; prevRB = curRB; prevLT = curLT; prevRT = curRT;
    }

    // ══════════════════════════════════════════════════════
    //  MOVE START
    // ══════════════════════════════════════════════════════

    private void startMove() {
        moveStart        = motor.getCurrentPosition();
        moveTarget       = moveStart + (rotateDir * TICKS_PER_SLOT);
        rotateDir        = -rotateDir;

        profileVelocity  = 0;
        profilePosition  = moveStart;
        moveStartMs      = System.currentTimeMillis();
        settleMs         = 0;
        peakPosError     = 0;
        peakVelError     = 0;
        overshot         = false;
        settled          = false;
        lastPosError     = Integer.MAX_VALUE;

        testState = TestState.MOVING;
    }

    // ══════════════════════════════════════════════════════
    //  kV MEASUREMENT START
    // ══════════════════════════════════════════════════════

    private void startKvMeasure() {
        kvMeasureStartMs = System.currentTimeMillis();
        kvSumVelocity    = 0;
        kvSampleCount    = 0;
        motor.setPower(KV_MEASURE_POWER);
        stateTimer.reset();
        testState = TestState.KV_MEASURING;
    }

    // ══════════════════════════════════════════════════════
    //  MAIN CONTROLLER — called every loop with actual dt
    // ══════════════════════════════════════════════════════

    private void runController(double dt) {
        int curPos = motor.getCurrentPosition();

        // Velocity estimation via finite difference
        estimatedVelocity = (curPos - lastEncoderPos) / Math.max(dt, 0.001);
        lastEncoderPos    = curPos;

        switch (testState) {

            case MOVING: {
                // ── 1. Advance trapezoidal profile ──────────────
                double distRemaining = moveTarget - profilePosition; // signed
                double direction     = Math.signum(distRemaining);

                // Stopping distance at current profile velocity
                double stopDist = (profileVelocity * profileVelocity) / (2.0 * maxAcceleration);

                if (Math.abs(distRemaining) <= stopDist + 1) {
                    // Decelerate
                    profileVelocity -= direction * maxAcceleration * dt;
                } else {
                    // Accelerate toward maxVelocity
                    profileVelocity += direction * maxAcceleration * dt;
                    profileVelocity  = clamp(profileVelocity, -maxVelocity, maxVelocity);
                }

                // Integrate profile position
                profilePosition += profileVelocity * dt;

                // ── 2. Compute errors ────────────────────────────
                int    posError = moveTarget - curPos;
                double velError = Math.abs(profileVelocity) - Math.abs(estimatedVelocity);

                // Track worst-case errors
                if (Math.abs(posError) > peakPosError) peakPosError = Math.abs(posError);

                // Only track velocity error during cruise (not during ramp up/down)
                boolean inCruise = Math.abs(profileVelocity) > maxVelocity * 0.9;
                if (inCruise && Math.abs(velError) > peakVelError) peakVelError = Math.abs(velError);

                // Overshoot detection — error sign flipped after approaching
                int signNow = (int) Math.signum(posError);
                if (lastPosError != Integer.MAX_VALUE && Math.signum(lastPosError) != signNow && signNow != 0) {
                    overshot = true;
                }
                lastPosError = posError;

                // ── 3. Command motor ─────────────────────────────
                double power = kV * profileVelocity + kP * posError;
                motor.setPower(clamp(power, -1.0, 1.0));

                // ── 4. Settle check ──────────────────────────────
                boolean profileDone = Math.abs(profileVelocity) < 5
                        && Math.abs(distRemaining) < SETTLED_TOLERANCE;
                if (!settled && profileDone && Math.abs(posError) <= SETTLED_TOLERANCE) {
                    settled   = true;
                    settleMs  = System.currentTimeMillis() - moveStartMs;
                    testState = TestState.COMPLETE;
                    recordRun();
                    motor.setPower(kP * posError); // gentle hold
                }

                // Hard timeout — 5 seconds
                if (System.currentTimeMillis() - moveStartMs > 5000) {
                    motor.setPower(0);
                    testState = TestState.COMPLETE;
                    recordRun();
                }
                break;
            }

            case KV_MEASURING: {
                // Collect velocity samples after a 0.3s spin-up
                long elapsed = System.currentTimeMillis() - kvMeasureStartMs;
                if (elapsed > 300) {
                    kvSumVelocity += Math.abs(estimatedVelocity);
                    kvSampleCount++;
                }
                if (elapsed >= (long)(KV_MEASURE_SECS * 1000)) {
                    motor.setPower(0);
                    if (kvSampleCount > 0) {
                        double measuredVelocity = kvSumVelocity / kvSampleCount;
                        kV = KV_MEASURE_POWER / measuredVelocity;
                        lastMeasuredKv = kV;
                    }
                    testState = TestState.COMPLETE;
                }
                break;
            }

            case IDLE:
            case COMPLETE:
                // Hold position with gentle P only
                int holdError = moveTarget - curPos;
                if (Math.abs(holdError) > SETTLED_TOLERANCE) {
                    motor.setPower(kP * holdError);
                } else {
                    motor.setPower(0);
                }
                break;
        }
    }

    // ══════════════════════════════════════════════════════
    //  LOG
    // ══════════════════════════════════════════════════════

    private void recordRun() {
        runNumber++;
        logLines.add(String.format(
                "R%02d | vMax=%4.0f | aMax=%5.0f | kV=%.6f | kP=%.5f | pErr_max=%3d | vErr_max=%5.1f | settle=%4dms | overshoot=%s",
                runNumber,
                maxVelocity, maxAcceleration,
                kV, kP,
                peakPosError,
                peakVelError,
                settleMs,
                overshot ? "YES" : "NO"
        ));
    }

    // ══════════════════════════════════════════════════════
    //  TELEMETRY
    // ══════════════════════════════════════════════════════

    private void showTelemetry() {
        if (showingLog) { showLog(); telemetry.update(); return; }

        int    curPos  = motor.getCurrentPosition();
        int    posError = moveTarget - curPos;

        telemetry.addLine("══ TRAPEZOIDAL TUNING ══════════════════════════");
        telemetry.addLine("");

        telemetry.addLine("─ CONSTANTS ─────────────────────────────────────");
        telemetry.addLine(String.format("  maxVelocity   %6.0f ticks/s     DPAD-L(−)  DPAD-R(+)", maxVelocity));
        telemetry.addLine(String.format("  maxAccel      %6.0f ticks/s²    LB(−)      RB(+)",     maxAcceleration));
        telemetry.addLine(String.format("  kV            %.6f          DPAD-D(−)  DPAD-U(+)",     kV));
        telemetry.addLine(String.format("  kP            %.5f           L-trig(−)  R-trig(+)",    kP));
        if (lastMeasuredKv >= 0) {
            telemetry.addLine(String.format("  [last kV measurement: %.6f]", lastMeasuredKv));
        }
        telemetry.addLine("");

        telemetry.addLine("─ LIVE ──────────────────────────────────────────");
        telemetry.addLine("  State:    " + stateLabel());
        telemetry.addLine(String.format("  Position: %d  (target %d  err %d)", curPos, moveTarget, posError));
        telemetry.addLine(String.format("  Vel actual:  %7.1f ticks/s", estimatedVelocity));
        telemetry.addLine(String.format("  Vel profile: %7.1f ticks/s", profileVelocity));
        telemetry.addLine(String.format("  Vel error:   %7.1f ticks/s", estimatedVelocity - profileVelocity));
        if (testState == TestState.MOVING) {
            long elapsed = System.currentTimeMillis() - moveStartMs;
            telemetry.addLine(String.format("  Elapsed: %dms  pErr_peak: %d  vErr_peak: %.1f",
                    elapsed, peakPosError, peakVelError));
        }
        if (testState == TestState.KV_MEASURING) {
            long elapsed = System.currentTimeMillis() - kvMeasureStartMs;
            telemetry.addLine(String.format("  Measuring kV... %.1f / %.1fs  samples: %d",
                    elapsed / 1000.0, KV_MEASURE_SECS, kvSampleCount));
        }
        telemetry.addLine("");

        telemetry.addLine("─ CONTROLS ──────────────────────────────────────");
        telemetry.addLine("  A=run move   X=measure kV   B=log   START=clear");
        telemetry.addLine("");

        int n = logLines.size();
        telemetry.addLine(String.format("─ LOG (%d runs) ──────────────────────────────────", runNumber));
        if (n > 0) {
            telemetry.addLine("  " + logLines.get(n - 1));
        } else {
            telemetry.addLine("  (no runs yet)");
        }

        telemetry.update();
    }

    private String stateLabel() {
        switch (testState) {
            case IDLE:         return "IDLE — press A to move, X to measure kV";
            case MOVING:       return "MOVING";
            case KV_MEASURING: return "MEASURING kV (do not touch motor)";
            case COMPLETE:     return "COMPLETE — press A or X";
            default:           return testState.toString();
        }
    }

    private void showLog() {
        int pages = (int) Math.ceil((double) logLines.size() / LINES_PER_PAGE);
        telemetry.addLine("══ LOG ═════════════════════════════════════════");
        telemetry.addLine(String.format("Page %d / %d    B=next/close    START=clear",
                logPage + 1, Math.max(1, pages)));
        telemetry.addLine("────────────────────────────────────────────────");

        if (logLines.isEmpty()) {
            telemetry.addLine("(no runs yet)");
        } else {
            int start = logPage * LINES_PER_PAGE;
            int end   = Math.min(start + LINES_PER_PAGE, logLines.size());
            for (int i = start; i < end; i++) telemetry.addLine(logLines.get(i));
        }

        telemetry.addLine("");
        telemetry.addLine("pErr_max  = worst position error during move  (target: < 30 ticks)");
        telemetry.addLine("vErr_max  = worst velocity error during cruise (target: < 50 ticks/s)");
        telemetry.addLine("settle_ms = time from start to within " + SETTLED_TOLERANCE + " ticks of target");
        telemetry.addLine("overshoot = position error changed sign after approaching target");
    }

    // ══════════════════════════════════════════════════════
    //  HELPERS
    // ══════════════════════════════════════════════════════

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}