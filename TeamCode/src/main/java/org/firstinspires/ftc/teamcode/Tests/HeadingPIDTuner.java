package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.threaded.CameraThread;
import org.firstinspires.ftc.teamcode.threaded.ControlHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.DriveThread;
import org.firstinspires.ftc.teamcode.threaded.ExpansionHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.SensorState;

/**
 * Heading PID Tuner with IMU-compensated bearing.
 *
 * The core problem: the camera only updates every ~30ms, so the PID reacts
 * to stale data and overshoots. This version uses the IMU (via currentPose
 * heading) to interpolate the bearing between camera frames, giving the PID
 * a fresh estimate every single loop (~10ms).
 *
 * ── CONTROLS ─────────────────────────────────────────────────────────────
 *
 *   Gamepad 1 (Driver):
 *     Left stick     — Drive (forward / strafe)
 *     Right stick X  — Manual rotate (when auto-align is OFF)
 *     Left bumper    — Toggle auto-align ON/OFF
 *
 *   Gamepad 2 (Tuner):
 *     D-Pad Up/Down  — Select coefficient (P / I / D / Deadband / Filter / MaxOut / Slew)
 *     Right bumper   — Increase selected coefficient
 *     Left bumper    — Decrease selected coefficient
 *     Right trigger  — Fine increase (hold)
 *     Left trigger   — Fine decrease (hold)
 *     A              — Reset I accumulator
 *     X              — Save current values to SensorState statics
 *     Y              — Toggle input filter ON/OFF
 *     B              — Toggle squared-P (non-linear) ON/OFF
 */
@TeleOp(name = "Align PID Tuner v2", group = "Tuning")
public class HeadingPIDTuner extends LinearOpMode {

    private static final int BASKET_TAG_ID = CameraThread.TAG_RED_BASKET;

    // ── Tunable PID coefficients ─────────────────────────────────────────
    private double kP       = SensorState.ALIGN_P;
    private double kI       = SensorState.ALIGN_I;
    private double kD       = SensorState.ALIGN_D;
    private double deadband = SensorState.ALIGN_DEADBAND;

    // ── Input filter ─────────────────────────────────────────────────────
    private boolean filterEnabled = true;
    private double filterAlpha    = 0.15;   // lower = more smoothing
    private double filteredBearing = 0;

    // ── Output limits ────────────────────────────────────────────────────
    private double maxOutput = 0.25;        // cap rotation power
    private double maxSlew   = 0.05;        // max output change per loop
    private double lastOutput = 0;

    // ── Non-linear P ─────────────────────────────────────────────────────
    private boolean squaredP = false;

    // ── PID state ────────────────────────────────────────────────────────
    private double integralSum = 0;
    private double lastError   = 0;
    private boolean hasLast    = false;
    private final ElapsedTime pidTimer = new ElapsedTime();

    // ── IMU compensation state ───────────────────────────────────────────
    //  Tracks the IMU heading at the moment the camera last updated,
    //  so we can compute how much the robot has rotated since then
    //  and adjust the stale camera bearing accordingly.
    private double lastRawBearing = Double.NaN;
    private double bearingAtLastCameraUpdate = 0;
    private double imuHeadingAtLastCameraUpdate = 0;

    // ── Tuner UI ─────────────────────────────────────────────────────────
    private enum TuneParam { P, I, D, DEADBAND, FILTER_ALPHA, MAX_OUTPUT, MAX_SLEW }
    private TuneParam selectedParam = TuneParam.P;

    private static final double STEP_P        = 0.001;
    private static final double STEP_I        = 0.0001;
    private static final double STEP_D        = 0.005;   // bigger steps since no /dt
    private static final double STEP_DEADBAND = 0.5;
    private static final double STEP_FILTER   = 0.05;
    private static final double STEP_MAXOUT   = 0.05;
    private static final double STEP_SLEW     = 0.01;
    private static final double FINE_DIVISOR  = 5.0;

    // Edge detection for button presses
    private boolean prevDpadUp2  = false, prevDpadDown2 = false;
    private boolean prevRB2 = false, prevLB2 = false;
    private boolean prevA2 = false, prevX2 = false, prevY2 = false, prevB2 = false;
    private boolean prevLB1 = false;

    // Error history for text graph
    private static final int GRAPH_WIDTH = 30;
    private final double[] errorHistory = new double[GRAPH_WIDTH];
    private int historyIndex = 0;

    // ── Threads ──────────────────────────────────────────────────────────
    private SensorState sensorState;
    private DriveThread driveThread;
    private CameraThread cameraThread;
    private ControlHubI2CThread controlHubI2C;
    private ExpansionHubI2CThread expansionHubI2C;

    @Override
    public void runOpMode() {
        sensorState = new SensorState(SensorState.Alliance.RED);

        driveThread     = new DriveThread(sensorState, hardwareMap);
        cameraThread    = new CameraThread(sensorState, hardwareMap, BASKET_TAG_ID);
        controlHubI2C   = new ControlHubI2CThread(sensorState, hardwareMap);
        expansionHubI2C = new ExpansionHubI2CThread(sensorState, hardwareMap);

        // Disable DriveThread's built-in auto-align — we control rotation here
        sensorState.setAutoAlignEnabled(false);

        telemetry.addLine("=== ALIGN PID TUNER v2 ===");
        telemetry.addLine("IMU-compensated bearing");
        telemetry.addLine("");
        telemetry.addLine("GP1 LB: toggle align");
        telemetry.addLine("GP2 D-Pad: select param");
        telemetry.addLine("GP2 Bumpers/Triggers: adjust");
        telemetry.addLine("GP2 Y: filter | B: squared-P");
        telemetry.update();

        waitForStart();

        driveThread.start();
        cameraThread.start();
        controlHubI2C.start();
        expansionHubI2C.start();
        pidTimer.reset();

        boolean localAlignEnabled = false;

        while (opModeIsActive()) {

            // ── Gamepad 1: Drive + toggle ────────────────────────────────
            double forward = -gamepad1.left_stick_y;
            double strafe  = -gamepad1.left_stick_x;
            double rotate  = -gamepad1.right_stick_x * 0.75;

            if (gamepad1.left_bumper && !prevLB1) {
                localAlignEnabled = !localAlignEnabled;
                if (!localAlignEnabled) {
                    resetPID();
                    lastOutput = 0;
                }
            }
            prevLB1 = gamepad1.left_bumper;

            // ── Get raw camera bearing and current IMU heading ───────────
            double rawBearing = sensorState.getTargetBearing();
            boolean tagVisible = sensorState.isBasketTagVisible();
            double imuHeading = Math.toDegrees(sensorState.getCurrentPose().getHeading());

            // ── IMU-compensated bearing ──────────────────────────────────
            //  When camera publishes a new bearing, we snapshot both the
            //  bearing and the IMU heading at that instant. Between camera
            //  frames, we use IMU delta to keep the bearing estimate fresh.
            double compensatedBearing = rawBearing;

            if (tagVisible) {
                // Detect new camera frame by checking if rawBearing changed
                if (Double.isNaN(lastRawBearing) || rawBearing != lastRawBearing) {
                    // New camera frame arrived — snapshot
                    bearingAtLastCameraUpdate = rawBearing;
                    imuHeadingAtLastCameraUpdate = imuHeading;
                    lastRawBearing = rawBearing;
                }

                // Compensate: if the robot has rotated since the last camera
                // frame, the real bearing has changed by that amount
                double imuDelta = imuHeading - imuHeadingAtLastCameraUpdate;
                compensatedBearing = bearingAtLastCameraUpdate - imuDelta;
            } else {
                // Tag lost — reset so next sighting is treated as fresh
                lastRawBearing = Double.NaN;
            }

            // ── Low-pass filter on the compensated bearing ───────────────
            double pidInput;
            if (filterEnabled && tagVisible) {
                filteredBearing = filteredBearing + filterAlpha * (compensatedBearing - filteredBearing);
                pidInput = filteredBearing;
            } else {
                filteredBearing = compensatedBearing;
                pidInput = compensatedBearing;
            }

            // ── Run PID ──────────────────────────────────────────────────
            double pidOutput = 0;
            if (localAlignEnabled && tagVisible) {
                pidOutput = runPID(pidInput);
                rotate = pidOutput;
            } else if (!tagVisible) {
                resetPID();
                lastOutput = 0;
            }

            sensorState.setDriveInput(forward, strafe, rotate);

            // ── Gamepad 2: Tuning controls ───────────────────────────────
            handleTunerInput();

            // ── Error history ────────────────────────────────────────────
            errorHistory[historyIndex] = tagVisible ? pidInput : 0;
            historyIndex = (historyIndex + 1) % GRAPH_WIDTH;

            // ── Telemetry ────────────────────────────────────────────────
            telemetry.addLine("=== ALIGN PID TUNER v2 (IMU comp) ===");
            telemetry.addData("Auto-Align", localAlignEnabled ? "ON" : "OFF");
            telemetry.addData("Tag Visible", tagVisible);
            telemetry.addLine("");

            telemetry.addData("Raw Bearing",        "%.2f°", rawBearing);
            telemetry.addData("Compensated Bearing", "%.2f°", compensatedBearing);
            telemetry.addData("Filtered (PID in)",   "%.2f°", pidInput);
            telemetry.addData("PID Output",          "%.4f", pidOutput);
            telemetry.addData("Range",               "%.1f in", sensorState.getTagRange());
            telemetry.addData("IMU Heading",         "%.2f°", imuHeading);
            telemetry.addLine("");

            telemetry.addData(paramLabel(TuneParam.P),            "%.6f", kP);
            telemetry.addData(paramLabel(TuneParam.I),            "%.6f", kI);
            telemetry.addData(paramLabel(TuneParam.D),            "%.6f", kD);
            telemetry.addData(paramLabel(TuneParam.DEADBAND),     "%.2f°", deadband);
            telemetry.addData(paramLabel(TuneParam.FILTER_ALPHA), "%.3f %s",
                    filterAlpha, filterEnabled ? "(ON)" : "(OFF)");
            telemetry.addData(paramLabel(TuneParam.MAX_OUTPUT),   "%.3f", maxOutput);
            telemetry.addData(paramLabel(TuneParam.MAX_SLEW),     "%.3f", maxSlew);
            telemetry.addLine("");
                telemetry.addData("Squared-P", squaredP ? "ON" : "OFF");
            telemetry.addLine("");

            telemetry.addLine(buildErrorGraph());
            telemetry.update();
        }

        sensorState.kill();
        joinAll();
    }

    // ── PID (no /dt on D term, with output cap and slew limiter) ─────────

    private double runPID(double error) {
        double dt = pidTimer.seconds();
        pidTimer.reset();

        // Inside deadband — stop and clear integral
        if (Math.abs(error) < deadband) {
            integralSum = 0;
            lastError = error;
            lastOutput = 0;
            return 0.0;
        }

        // ── P term ───────────────────────────────────────────────────────
        double p;
        if (squaredP) {
            // Non-linear: aggressive far away, gentle near setpoint
            p = kP * error * Math.abs(error);
        } else {
            p = kP * error;
        }

        // ── I term ───────────────────────────────────────────────────────
        if (dt > 0 && dt < 1.0) {
            integralSum += error * dt;
            integralSum = clamp(integralSum, -0.3, 0.3);
        }
        double i = kI * integralSum;

        // ── D term (no /dt — just delta error) ──────────────────────────
        double d = 0;
        if (hasLast) {
            d = kD * (error - lastError);
        }

        lastError = error;
        hasLast = true;

        // ── Output cap ───────────────────────────────────────────────────
        double raw = clamp(p + i + d, -maxOutput, maxOutput);

        // ── Slew rate limiter ────────────────────────────────────────────
        //  Prevents sudden jumps in motor power that cause mechanical jerk
        //  and make stale-data oscillation worse.
        double limited = clamp(raw, lastOutput - maxSlew, lastOutput + maxSlew);
        lastOutput = limited;

        return limited;
    }

    private void resetPID() {
        integralSum = 0;
        lastError = 0;
        hasLast = false;
        pidTimer.reset();
    }

    // ── Tuner input handling ─────────────────────────────────────────────

    private void handleTunerInput() {
        // Select parameter
        if (gamepad2.dpad_up && !prevDpadUp2) selectedParam = prevParam(selectedParam);
        prevDpadUp2 = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !prevDpadDown2) selectedParam = nextParam(selectedParam);
        prevDpadDown2 = gamepad2.dpad_down;

        // Coarse adjust
        if (gamepad2.right_bumper && !prevRB2) adjustParam(selectedParam, +1, false);
        prevRB2 = gamepad2.right_bumper;

        if (gamepad2.left_bumper && !prevLB2) adjustParam(selectedParam, -1, false);
        prevLB2 = gamepad2.left_bumper;

        // Fine adjust (held)
        if (gamepad2.right_trigger > 0.3) adjustParam(selectedParam, +1, true);
        if (gamepad2.left_trigger > 0.3)  adjustParam(selectedParam, -1, true);

        // A — reset integral
        if (gamepad2.a && !prevA2) integralSum = 0;
        prevA2 = gamepad2.a;

        // X — save to statics
        if (gamepad2.x && !prevX2) {
            SensorState.ALIGN_P = kP;
            SensorState.ALIGN_I = kI;
            SensorState.ALIGN_D = kD;
            SensorState.ALIGN_DEADBAND = deadband;
        }
        prevX2 = gamepad2.x;

        // Y — toggle filter
        if (gamepad2.y && !prevY2) filterEnabled = !filterEnabled;
        prevY2 = gamepad2.y;

        // B — toggle squared P
        if (gamepad2.b && !prevB2) squaredP = !squaredP;
        prevB2 = gamepad2.b;
    }

    private void adjustParam(TuneParam param, int dir, boolean fine) {
        double div = fine ? FINE_DIVISOR : 1.0;
        switch (param) {
            case P:
                kP = Math.max(0, kP + dir * STEP_P / div);
                break;
            case I:
                kI = Math.max(0, kI + dir * STEP_I / div);
                break;
            case D:
                kD = Math.max(0, kD + dir * STEP_D / div);
                break;
            case DEADBAND:
                deadband = Math.max(0, deadband + dir * STEP_DEADBAND / div);
                break;
            case FILTER_ALPHA:
                filterAlpha = clamp(filterAlpha + dir * STEP_FILTER / div, 0.05, 1.0);
                break;
            case MAX_OUTPUT:
                maxOutput = clamp(maxOutput + dir * STEP_MAXOUT / div, 0.05, 1.0);
                break;
            case MAX_SLEW:
                maxSlew = clamp(maxSlew + dir * STEP_SLEW / div, 0.005, 0.5);
                break;
        }
    }

    // ── UI helpers ───────────────────────────────────────────────────────

    private String paramLabel(TuneParam param) {
        String prefix = (param == selectedParam) ? ">> " : "   ";
        switch (param) {
            case P:            return prefix + "kP";
            case I:            return prefix + "kI";
            case D:            return prefix + "kD";
            case DEADBAND:     return prefix + "Deadband";
            case FILTER_ALPHA: return prefix + "Filter α";
            case MAX_OUTPUT:   return prefix + "Max Output";
            case MAX_SLEW:     return prefix + "Max Slew";
            default:           return prefix + "?";
        }
    }

    private String buildErrorGraph() {
        StringBuilder sb = new StringBuilder();
        sb.append("Error History (±20°):\n");
        double scale = 20.0;
        int barHalf = 10;

        for (int i = 0; i < GRAPH_WIDTH; i++) {
            int idx = (historyIndex + i) % GRAPH_WIDTH;
            double err = errorHistory[idx];
            int offset = (int) Math.round((err / scale) * barHalf);
            offset = Math.max(-barHalf, Math.min(barHalf, offset));

            char[] bar = new char[barHalf * 2 + 1];
            for (int j = 0; j < bar.length; j++) bar[j] = ' ';
            bar[barHalf] = '|';
            if (offset != 0) bar[barHalf + offset] = '*';

            sb.append(new String(bar));
            if (i < GRAPH_WIDTH - 1) sb.append('\n');
        }
        return sb.toString();
    }

    private TuneParam nextParam(TuneParam p) {
        TuneParam[] v = TuneParam.values();
        return v[(p.ordinal() + 1) % v.length];
    }

    private TuneParam prevParam(TuneParam p) {
        TuneParam[] v = TuneParam.values();
        return v[(p.ordinal() - 1 + v.length) % v.length];
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private void joinAll() {
        try {
            driveThread.join(200);
            cameraThread.join(200);
            controlHubI2C.join(200);
            expansionHubI2C.join(200);
        } catch (InterruptedException ignored) {}
    }
}