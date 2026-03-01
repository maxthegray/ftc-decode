package org.firstinspires.ftc.teamcode.Tests.NewCarousel;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/**
 * ═══════════════════════════════════════════════════════════════════════════
 *  INTEGRATED CYCLE TIMING CALIBRATION  (trapezoidal profile edition)
 *  ─────────────────────────────────────────────────────────────────────────
 *  Run TrapezoidalTuningTest FIRST to find kV, kP, maxVelocity, maxAcceleration.
 *  Copy those values into the four constants at the top of this file,
 *  then use this test to find minimum kicker dwell and safety delay.
 *
 *  Two cycle types:
 *    A  → ROTATE + KICK  (carousel rotates 1 slot, then kicks)
 *    X  → KICK ONLY      (no rotation — ball already at intake)
 *
 *  ─── CONTROLS ────────────────────────────────────────────────────────────
 *
 *    A              → ROTATE + KICK cycle  (alternates L/R each press)
 *    X              → KICK ONLY cycle
 *    B              → View / page log
 *    START          → Clear log + reset counters
 *
 *  Tune between runs (one button per constant):
 *
 *    DPAD L / R     → maxVelocity    − / +  (step 50 ticks/s)
 *    LB / RB        → maxAccel       − / +  (step 200 ticks/s²)
 *    DPAD D / U     → Kicker dwell   − / +  (step 5 ms)
 *    L-trig/R-trig  → Safety delay   − / +  (step 5 ms)  [pull > 0.5]
 *    Y + DPAD L/R   → tolerance      − / +  (step 1 tick) [hold Y]
 *
 *  ─── CONSTANTS TO SET FROM TrapezoidalTuningTest ─────────────────────────
 *  Edit the four values below before running this test.
 *
 *  ─── LOG FORMAT ──────────────────────────────────────────────────────────
 *
 *  ROTATE+KICK — two lines:
 *    C## [ROTATE] | vMax=XXXX | aMax=XXXXX | tol=XX | settle=XXXms | err=X
 *    K## [ROTATE] | dwell=XXXms | safety=XXXms | V_trig=X.XXV | V_peak=X.XXV |
 *                   fired=YES/NO | t_ret=XXms | rec_safety=XXms | total=XXXms
 *
 *  KICK ONLY — one line:
 *    K## [KICK  ] | dwell=XXXms | safety=XXXms | V_trig=X.XXV | V_peak=X.XXV |
 *                   fired=YES/NO | t_ret=XXms | rec_safety=XXms | total=XXXms
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */
@Disabled
@TeleOp(name = "Timing Calibration Test", group = "Test")
public class TimingCalibrationTest extends LinearOpMode {

    // ══════════════════════════════════════════════════════
    //  ★ SET THESE FROM TrapezoidalTuningTest RESULTS ★
    // ══════════════════════════════════════════════════════

    private double kV  = 0.0007;  // from tuning test STEP 0 (kV measurement)
    private double kP  = 0.0003;  // from tuning test STEP 3

    // ══════════════════════════════════════════════════════
    //  HARDWARE
    // ══════════════════════════════════════════════════════

    private DcMotorEx   carouselMotor;
    private Servo       kickerServo;
    private AnalogInput kickerFeedback;

    // ══════════════════════════════════════════════════════
    //  TUNABLE CONSTANTS (adjusted at runtime)
    // ══════════════════════════════════════════════════════

    private double maxVelocity     = 1000.0; // ticks/s  — from TrapezoidalTuningTest
    private double maxAcceleration = 3000.0; // ticks/s² — from TrapezoidalTuningTest
    private int    tolerance       = 8;      // ticks — settle threshold
    private int    dwellMs         = 100;    // kicker up dwell (ms)
    private int    safetyDelayMs   = 200;    // kicker safety delay after DOWN (ms)

    // ══════════════════════════════════════════════════════
    //  KICKER CONSTANTS
    // ══════════════════════════════════════════════════════

    private static final double KICKER_POS_UP          = 0.3;
    private static final double KICKER_POS_DOWN        = 0.0;
    private static final double DOWN_VOLTAGE_THRESHOLD = 1.3;
    private static final double KICK_DETECT_DELTA      = 0.4;  // V rise to confirm kick

    // ══════════════════════════════════════════════════════
    //  FIXED CAROUSEL CONSTANTS
    // ══════════════════════════════════════════════════════

    private static final int TICKS_PER_SLOT = 2731;

    // ══════════════════════════════════════════════════════
    //  CYCLE STATE MACHINE
    // ══════════════════════════════════════════════════════

    private enum CycleState {
        IDLE,
        CAROUSEL_MOVING,    // Trapezoidal profile running
        KICKER_GOING_UP,    // UP commanded, counting dwell
        KICKER_GOING_DOWN,  // DOWN commanded, waiting return + safety
        CYCLE_COMPLETE
    }

    private CycleState state        = CycleState.IDLE;
    private boolean    isRotateCycle = true;

    // Carousel profile runtime
    private double profileVelocity  = 0;
    private double profilePosition  = 0;
    private int    carouselTarget   = 0;
    private int    carouselStart    = 0;
    private int    rotateDir        = 1;
    private int    lastEncoderPos   = 0;

    // Per-cycle timing
    private long cycleStartMs       = 0;
    private long settleDoneMs       = 0;
    private int  finalCarouselError = 0;

    // Kicker runtime
    private long   kickDownCmdMs    = 0;
    private long   kickReturnMs     = 0;
    private double voltageAtTrigger = 0;
    private double peakVoltage      = 0;
    private boolean kickFired       = false;

    // ══════════════════════════════════════════════════════
    //  LOG
    // ══════════════════════════════════════════════════════

    private final List<String> logLines       = new ArrayList<>();
    private int rotateRunNumber   = 0;
    private int kickOnlyRunNumber = 0;
    private boolean showingLog    = false;
    private int     logPage       = 0;
    private static final int LINES_PER_PAGE = 10;

    // ══════════════════════════════════════════════════════
    //  TIMERS + EDGE DETECTION
    // ══════════════════════════════════════════════════════

    private final ElapsedTime loopTimer  = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();

    private boolean prevA  = false, prevX  = false;
    private boolean prevB  = false, prevSt = false;
    private boolean prevY  = false;
    private boolean prevDL = false, prevDR = false;
    private boolean prevDU = false, prevDD = false;
    private boolean prevLB = false, prevRB = false;
    private boolean prevLT = false, prevRT = false;

    // ══════════════════════════════════════════════════════
    //  ENTRY POINT
    // ══════════════════════════════════════════════════════

    @Override
    public void runOpMode() {
        carouselMotor  = hardwareMap.get(DcMotorEx.class,   "carousel_motor");
        kickerServo    = hardwareMap.get(Servo.class,        "flicker_servo");
        kickerFeedback = hardwareMap.get(AnalogInput.class,  "flick");

        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // we control power directly
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        kickerServo.setPosition(KICKER_POS_DOWN);

        lastEncoderPos = carouselMotor.getCurrentPosition();
        carouselTarget = lastEncoderPos; // hold in place from start

        telemetry.addLine("TIMING CALIBRATION — Ready.  START on field controller.");
        telemetry.addLine("Run TrapezoidalTuningTest first to set kV, kP, maxVelocity, maxAccel.");
        telemetry.update();
        waitForStart();

        loopTimer.reset();

        while (opModeIsActive()) {
            double dt = Math.min(loopTimer.seconds(), 0.05);
            loopTimer.reset();

            handleInput();
            runStateMachine(dt);
            showTelemetry();
            sleep(10);
        }

        carouselMotor.setPower(0);
        kickerServo.setPosition(KICKER_POS_DOWN);
    }

    // ══════════════════════════════════════════════════════
    //  INPUT
    // ══════════════════════════════════════════════════════

    private void handleInput() {
        boolean curA  = gamepad1.a;
        boolean curX  = gamepad1.x;
        boolean curB  = gamepad1.b;
        boolean curY  = gamepad1.y;
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
            logLines.clear();
            rotateRunNumber = kickOnlyRunNumber = 0;
            showingLog = false;
        }

        boolean canAct = (state == CycleState.IDLE || state == CycleState.CYCLE_COMPLETE)
                && !showingLog;

        if (canAct) {
            if (curA && !prevA) startCycle(true);
            if (curX && !prevX) startCycle(false);

            boolean holdingY = curY;

            if (holdingY) {
                // Y modifier: tolerance
                if (curDL && !prevDL) tolerance = Math.max(3,  tolerance - 1);
                if (curDR && !prevDR) tolerance = Math.min(30, tolerance + 1);
            } else {
                // Normal: maxVelocity — DPAD L/R
                if (curDL && !prevDL) maxVelocity = Math.max(100, maxVelocity - 50);
                if (curDR && !prevDR) maxVelocity = maxVelocity + 50;

                // maxAcceleration — LB / RB
                if (curLB && !prevLB) maxAcceleration = Math.max(500, maxAcceleration - 200);
                if (curRB && !prevRB) maxAcceleration = maxAcceleration + 200;

                // Kicker dwell — DPAD D / U
                if (curDD && !prevDD) dwellMs = Math.max(30,  dwellMs - 5);
                if (curDU && !prevDU) dwellMs = Math.min(300, dwellMs + 5);

                // Safety delay — L-trig / R-trig
                if (curLT && !prevLT) safetyDelayMs = Math.max(30,  safetyDelayMs - 5);
                if (curRT && !prevRT) safetyDelayMs = Math.min(500, safetyDelayMs + 5);
            }
        }

        prevA  = curA;  prevX  = curX;  prevY  = curY;
        prevB  = curB;  prevSt = curSt;
        prevDL = curDL; prevDR = curDR; prevDU = curDU; prevDD = curDD;
        prevLB = curLB; prevRB = curRB; prevLT = curLT; prevRT = curRT;
    }

    // ══════════════════════════════════════════════════════
    //  CYCLE START
    // ══════════════════════════════════════════════════════

    private void startCycle(boolean rotate) {
        isRotateCycle    = rotate;
        cycleStartMs     = System.currentTimeMillis();
        settleDoneMs     = 0;
        finalCarouselError = 0;
        kickDownCmdMs    = 0;
        kickReturnMs     = 0;
        peakVoltage      = 0;
        kickFired        = false;
        voltageAtTrigger = kickerFeedback.getVoltage();

        if (rotate) {
            carouselStart    = carouselMotor.getCurrentPosition();
            carouselTarget   = carouselStart + (rotateDir * TICKS_PER_SLOT);
            rotateDir        = -rotateDir;
            profileVelocity  = 0;
            profilePosition  = carouselStart;
            lastEncoderPos   = carouselStart;
            stateTimer.reset();
            state = CycleState.CAROUSEL_MOVING;
        } else {
            // Kick immediately
            settleDoneMs = 0;
            kickerServo.setPosition(KICKER_POS_UP);
            stateTimer.reset();
            state = CycleState.KICKER_GOING_UP;
        }
    }

    // ══════════════════════════════════════════════════════
    //  STATE MACHINE
    // ══════════════════════════════════════════════════════

    private void runStateMachine(double dt) {
        int    curPos  = carouselMotor.getCurrentPosition();
        long   elapsed = System.currentTimeMillis() - cycleStartMs;
        double volt    = kickerFeedback.getVoltage();

        if (volt > peakVoltage) peakVoltage = volt;

        switch (state) {

            // ── Trapezoidal profile ───────────────────────────────
            case CAROUSEL_MOVING: {
                double distRemaining = carouselTarget - profilePosition;
                double direction     = Math.signum(distRemaining);
                double stopDist      = (profileVelocity * profileVelocity) / (2.0 * maxAcceleration);

                if (Math.abs(distRemaining) <= stopDist + 1) {
                    profileVelocity -= direction * maxAcceleration * dt;
                } else {
                    profileVelocity += direction * maxAcceleration * dt;
                    profileVelocity  = clamp(profileVelocity, -maxVelocity, maxVelocity);
                }

                profilePosition += profileVelocity * dt;

                int    posError = carouselTarget - curPos;
                double power    = kV * profileVelocity + kP * posError;
                carouselMotor.setPower(clamp(power, -1.0, 1.0));

                // Settle check: profile nearly stopped and position within tolerance
                boolean profileDone = Math.abs(profileVelocity) < 5
                        && Math.abs(distRemaining) < tolerance;
                if (profileDone && Math.abs(posError) <= tolerance) {
                    carouselMotor.setPower(0);
                    settleDoneMs       = elapsed;
                    finalCarouselError = posError;

                    // Immediately kick
                    kickerServo.setPosition(KICKER_POS_UP);
                    stateTimer.reset();
                    state = CycleState.KICKER_GOING_UP;
                }

                // Hard timeout
                if (elapsed > 5000) {
                    carouselMotor.setPower(0);
                    state = CycleState.CYCLE_COMPLETE;
                    recordRun(elapsed);
                }
                break;
            }

            // ── Kicker going up / dwell ───────────────────────────
            case KICKER_GOING_UP:
                if (!kickFired && volt > voltageAtTrigger + KICK_DETECT_DELTA) {
                    kickFired = true;
                }
                if (stateTimer.milliseconds() >= dwellMs) {
                    kickDownCmdMs = elapsed;
                    kickerServo.setPosition(KICKER_POS_DOWN);
                    stateTimer.reset();
                    state = CycleState.KICKER_GOING_DOWN;
                }
                // Hold carousel during kick
                if (isRotateCycle) {
                    int holdErr = carouselTarget - curPos;
                    carouselMotor.setPower(Math.abs(holdErr) > tolerance ? kP * holdErr : 0);
                }
                break;

            // ── Kicker returning + safety delay ───────────────────
            case KICKER_GOING_DOWN: {
                boolean physDown     = volt <= DOWN_VOLTAGE_THRESHOLD;
                boolean safetyPassed = stateTimer.milliseconds() >= safetyDelayMs;

                if (physDown && kickReturnMs == 0) {
                    kickReturnMs = elapsed - kickDownCmdMs;
                }
                if (physDown && safetyPassed) {
                    state = CycleState.CYCLE_COMPLETE;
                    recordRun(elapsed);
                }
                if (stateTimer.milliseconds() > 3000) {
                    state = CycleState.CYCLE_COMPLETE;
                    recordRun(elapsed);
                }
                // Hold carousel
                if (isRotateCycle) {
                    int holdErr = carouselTarget - curPos;
                    carouselMotor.setPower(Math.abs(holdErr) > tolerance ? kP * holdErr : 0);
                }
                break;
            }

            // ── Idle / complete: hold carousel position ───────────
            case IDLE:
            case CYCLE_COMPLETE: {
                int holdErr = carouselTarget - curPos;
                carouselMotor.setPower(Math.abs(holdErr) > tolerance ? kP * holdErr : 0);
                break;
            }
        }

        lastEncoderPos = curPos;
    }

    // ══════════════════════════════════════════════════════
    //  LOG A COMPLETED RUN
    // ══════════════════════════════════════════════════════

    private void recordRun(long totalMs) {
        long recSafety = (kickReturnMs > 0) ? (kickReturnMs + 20) : -1;
        String safetyStr = recSafety > 0 ? (recSafety + "ms") : "N/A";
        String retStr    = kickReturnMs > 0 ? (kickReturnMs + "ms") : "N/A";

        if (isRotateCycle) {
            rotateRunNumber++;
            logLines.add(String.format(
                    "C%02d [ROTATE] | vMax=%4.0f | aMax=%5.0f | tol=%2d | settle=%3dms | err=%d",
                    rotateRunNumber, maxVelocity, maxAcceleration, tolerance,
                    settleDoneMs, finalCarouselError
            ));
            logLines.add(String.format(
                    "K%02d [ROTATE] | dwell=%3dms | safety=%3dms | V_trig=%.2fV | V_peak=%.2fV | fired=%-3s | t_ret=%s | rec_safety=%s | total=%dms",
                    rotateRunNumber, dwellMs, safetyDelayMs,
                    voltageAtTrigger, peakVoltage,
                    kickFired ? "YES" : "NO",
                    retStr, safetyStr, totalMs
            ));
        } else {
            kickOnlyRunNumber++;
            logLines.add(String.format(
                    "K%02d [KICK  ] | dwell=%3dms | safety=%3dms | V_trig=%.2fV | V_peak=%.2fV | fired=%-3s | t_ret=%s | rec_safety=%s | total=%dms",
                    kickOnlyRunNumber, dwellMs, safetyDelayMs,
                    voltageAtTrigger, peakVoltage,
                    kickFired ? "YES" : "NO",
                    retStr, safetyStr, totalMs
            ));
        }
    }

    // ══════════════════════════════════════════════════════
    //  TELEMETRY
    // ══════════════════════════════════════════════════════

    private void showTelemetry() {
        if (showingLog) { showLog(); telemetry.update(); return; }

        long   elapsed = System.currentTimeMillis() - cycleStartMs;
        int    curPos  = carouselMotor.getCurrentPosition();
        double volt    = kickerFeedback.getVoltage();

        telemetry.addLine("══ TIMING CALIBRATION ══════════════════════════");
        telemetry.addLine("");

        telemetry.addLine("─ CONSTANTS ─────────────────────────────────────");
        telemetry.addLine(String.format("  maxVelocity  %5.0f ticks/s     DPAD-L(−)  DPAD-R(+)", maxVelocity));
        telemetry.addLine(String.format("  maxAccel     %5.0f ticks/s²    LB(−)      RB(+)",     maxAcceleration));
        telemetry.addLine(String.format("  tolerance     %2d ticks         Y+DPAD-L(−)  Y+DPAD-R(+)", tolerance));
        telemetry.addLine(String.format("  dwell        %4d ms            DPAD-D(−)  DPAD-U(+)", dwellMs));
        telemetry.addLine(String.format("  safety       %4d ms            L-trig(−)  R-trig(+)", safetyDelayMs));
        telemetry.addLine(String.format("  kV=%.6f  kP=%.5f  (set from TrapezoidalTuningTest)", kV, kP));
        telemetry.addLine("");

        telemetry.addLine("─ LIVE ──────────────────────────────────────────");
        telemetry.addLine("  State:    " + stateLabel());
        telemetry.addLine(String.format("  Voltage:  %.3fV   peak %.3fV", volt, peakVoltage));
        if (state != CycleState.IDLE && cycleStartMs > 0) {
            telemetry.addLine(String.format("  Elapsed:  %dms", elapsed));
            telemetry.addLine(String.format("  Crsl:     pos=%d  target=%d  err=%d",
                    curPos, carouselTarget, carouselTarget - curPos));
            telemetry.addLine(String.format("  Profile:  vel=%.1f", profileVelocity));
        }
        if (state == CycleState.KICKER_GOING_DOWN) {
            telemetry.addLine(String.format("  Safety:   %.0f / %dms", stateTimer.milliseconds(), safetyDelayMs));
        }
        telemetry.addLine("");

        telemetry.addLine("─ CONTROLS ──────────────────────────────────────");
        telemetry.addLine("  A=ROTATE+KICK   X=KICK ONLY   B=log   START=clear");
        telemetry.addLine("  Hold Y to adjust tolerance with DPAD-L/R");
        telemetry.addLine("");

        int n = logLines.size();
        telemetry.addLine(String.format("─ LOG (%dR %dK) ──────────────────────────────────",
                rotateRunNumber, kickOnlyRunNumber));
        if (n >= 1) {
            int previewStart = Math.max(0, n - 2);
            for (int i = previewStart; i < n; i++) telemetry.addLine("  " + logLines.get(i));
        } else {
            telemetry.addLine("  (no runs yet)");
        }

        telemetry.update();
    }

    private String stateLabel() {
        switch (state) {
            case IDLE:           return "IDLE — press A or X";
            case CYCLE_COMPLETE: return "DONE — press A or X";
            case CAROUSEL_MOVING:return "CAROUSEL moving (trapezoidal)";
            case KICKER_GOING_UP:return "KICKER UP (dwell)";
            case KICKER_GOING_DOWN: return "KICKER returning";
            default: return state.toString();
        }
    }

    private void showLog() {
        int pages = (int) Math.ceil((double) logLines.size() / LINES_PER_PAGE);
        telemetry.addLine("══ RUN LOG ═════════════════════════════════════");
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
        telemetry.addLine("C row: settle=carousel settle time | err=final tick error");
        telemetry.addLine("K row: t_ret=servo return time | rec_safety=suggested min | total=full cycle");
    }

    // ══════════════════════════════════════════════════════
    //  HELPERS
    // ══════════════════════════════════════════════════════

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}