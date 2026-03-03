package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.threaded.ControlHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.ExpansionHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.MechanismThread;
import org.firstinspires.ftc.teamcode.threaded.PipelineEventLog;
import org.firstinspires.ftc.teamcode.threaded.SensorState;
import org.firstinspires.ftc.teamcode.threaded.ShootSequence;

/**
 * ═══════════════════════════════════════════════════════════════════════════
 *  PIPELINE TIMER TEST — measure the full intake→index→ready pipeline
 * ───────────────────────────────────────────────────────────────────────────
 *  No driving. Feed balls in by hand. Logs every event with millisecond
 *  precision to a CSV file on the phone.
 *
 *  HOW TO USE:
 *    1. Start this OpMode
 *    2. Press Right Bumper to turn intake ON
 *    3. Feed balls one at a time into the intake
 *    4. Watch telemetry — it shows live event timestamps
 *    5. Press Left Bumper to save the log file and see the summary
 *    6. Pull /sdcard/FIRST/pipeline_log_XXXX.csv off the phone for analysis
 *
 *  CONTROLS:
 *    Right Bumper (GP1) — Toggle intake ON/OFF
 *    Left Bumper  (GP1) — Save log + show summary
 *    A            (GP1) — Toggle skipKickback (shows current mode)
 *    B            (GP1) — Emergency stop intake
 *    Right Trigger(GP1) — Manual intake IN (bypasses toggle)
 *    Left Trigger (GP1) — Manual intake OUT (unjam)
 *    Y            (GP1) — Reset log (clear all events, start fresh)
 *
 *  TIPS:
 *    - Run once with skipKickback=false, then once with true, to compare
 *    - Feed balls slowly (wait for "IDLE" between each) for clean data
 *    - The summary shows per-ball cycle breakdowns
 * ═══════════════════════════════════════════════════════════════════════════
 */
@TeleOp(name = "Pipeline Timer Test", group = "Test")
public class PipelineTimerTest extends OpMode {

    // ════════════════════════════════════════════════════════════════════════
    //  STATE
    // ════════════════════════════════════════════════════════════════════════

    private MechanismThread       mechanismThread;
    private ControlHubI2CThread   controlHubI2C;
    private ExpansionHubI2CThread expansionHubI2C;
    private SensorState           sensorState;
    private AnalogInput           rampSensor1;
    private AnalogInput           rampSensor2;
    private PipelineEventLog      pipelineLog;

    private boolean intakeOn        = false;
    private boolean skipKickback    = true;  // default to auto-mode (skip kickback)
    private boolean logSaved        = false;
    private String  savedFilePath   = "";
    private String  savedSummary    = "";
    private int     runNumber       = 0;

    // Edge detection for buttons
    private boolean prevRB = false;
    private boolean prevLB = false;
    private boolean prevA  = false;
    private boolean prevY  = false;

    // ════════════════════════════════════════════════════════════════════════
    //  LIFECYCLE
    // ════════════════════════════════════════════════════════════════════════

    @Override
    public void init() {
        sensorState = new SensorState(SensorState.Alliance.BLUE);

        mechanismThread = new MechanismThread(hardwareMap);
        mechanismThread.setSensorState(sensorState);
        mechanismThread.setSkipKickback(skipKickback);

        controlHubI2C   = new ControlHubI2CThread(sensorState, hardwareMap);
        expansionHubI2C = new ExpansionHubI2CThread(sensorState, hardwareMap);

        rampSensor1 = hardwareMap.get(AnalogInput.class, "touch1");
        rampSensor2 = hardwareMap.get(AnalogInput.class, "touch2");

        pipelineLog = new PipelineEventLog();
        mechanismThread.setPipelineLog(pipelineLog);

        mechanismThread.enqueueCommand(
                new MechanismThread.Command(MechanismThread.Command.Type.SET_AUTO_INDEX, true));

        telemetry.addData("Status", "Initialized — press Start, then RB to begin intake");
        telemetry.addData("Kickback", skipKickback ? "SKIPPED (auto mode)" : "ENABLED (teleop mode)");
        telemetry.update();
    }

    @Override
    public void start() {
        mechanismThread.start();
        controlHubI2C.start();
        expansionHubI2C.start();
    }

    @Override
    public void loop() {
        // ── Update mechanism thread with latest sensor data ───────────────
        mechanismThread.setBallPositions(sensorState.getAllPositions());

        // ── Controls ──────────────────────────────────────────────────────

        // Right Bumper — toggle intake
        if (gamepad1.right_bumper && !prevRB) {
            intakeOn = !intakeOn;
            logSaved = false; // new data incoming, un-save
        }
        prevRB = gamepad1.right_bumper;

        // Left Bumper — save log
        if (gamepad1.left_bumper && !prevLB) {
            saveLog();
        }
        prevLB = gamepad1.left_bumper;

        // A — toggle skipKickback
        if (gamepad1.a && !prevA) {
            skipKickback = !skipKickback;
            mechanismThread.setSkipKickback(skipKickback);
        }
        prevA = gamepad1.a;

        // B — emergency stop
        if (gamepad1.b) {
            intakeOn = false;
        }

        // Y — reset log
        if (gamepad1.y && !prevY) {
            pipelineLog = new PipelineEventLog();
            mechanismThread.setPipelineLog(pipelineLog);
            logSaved = false;
            savedSummary = "";
        }
        prevY = gamepad1.y;

        // Manual override triggers
        if (gamepad1.right_trigger > 0.1) {
            mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);
        } else if (gamepad1.left_trigger > 0.1) {
            mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.OUT);
        } else if (intakeOn) {
            mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);
        } else {
            mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
        }

        // ── Telemetry ─────────────────────────────────────────────────────
        updateTelemetry();
    }

    @Override
    public void stop() {
        // Auto-save on stop if not already saved
        if (!logSaved && pipelineLog.getCount() > 0) {
            saveLog();
        }

        sensorState.kill();
        mechanismThread.kill();

        try { mechanismThread.join(500); } catch (InterruptedException ignored) {}
    }

    // ════════════════════════════════════════════════════════════════════════
    //  LOG MANAGEMENT
    // ════════════════════════════════════════════════════════════════════════

    private void saveLog() {
        runNumber++;
        String filename = "pipeline_log_" + String.format("%03d", runNumber);
        savedFilePath = pipelineLog.writeToFile(filename);
        savedSummary  = pipelineLog.getSummary();
        logSaved = true;
    }

    // ════════════════════════════════════════════════════════════════════════
    //  TELEMETRY
    // ════════════════════════════════════════════════════════════════════════

    private void updateTelemetry() {
        double r1V = rampSensor1.getVoltage();
        double r2V = rampSensor2.getVoltage();
        boolean rampTriggered = sensorState.isRampTriggered();

        ShootSequence.BallColor[] pos = sensorState.getAllPositions();

        telemetry.addLine("══ PIPELINE TIMER TEST ══════════════════════════");
        telemetry.addLine("");

        // Status
        telemetry.addData("Intake", intakeOn ? "ON ◄" : "off");
        telemetry.addData("Kickback", skipKickback ? "SKIPPED (auto)" : "ENABLED (teleop)");
        telemetry.addData("Mechanism", mechanismThread.isIdle() ? "IDLE" : mechanismThread.getStateDebug());
        telemetry.addLine("");

        // Sensors
        telemetry.addData("Ramp1 (lower)", "%.3fV  %s", r1V,
                r1V >= SensorState.RAMP_SENSOR_THRESHOLD ? "◄ TRIG" : "");
        telemetry.addData("Ramp2 (upper)", "%.3fV  %s", r2V,
                r2V >= SensorState.RAMP_SENSOR_THRESHOLD ? "◄ TRIG" : "");
        telemetry.addData("Ramp Combined", rampTriggered ? "TRIGGERED ◄" : "clear");
        telemetry.addLine("");

        // Ball positions
        telemetry.addData("Balls", "intake=%s  BL=%s  BR=%s",
                shortName(pos[0]), shortName(pos[1]), shortName(pos[2]));
        telemetry.addLine("");

        // Log status
        telemetry.addData("Events recorded", "%d / 4000%s",
                pipelineLog.getCount(),
                pipelineLog.isFull() ? "  ◄ FULL" : "");

        if (logSaved) {
            telemetry.addLine("");
            telemetry.addData("Saved to", savedFilePath);
            telemetry.addLine("");
            // Show just the first ~15 lines of summary (telemetry has limited space)
            String[] lines = savedSummary.split("\n");
            int maxLines = Math.min(lines.length, 20);
            for (int i = 0; i < maxLines; i++) {
                telemetry.addLine(lines[i]);
            }
            if (lines.length > maxLines) {
                telemetry.addLine("  ... (see CSV for full data)");
            }
        }

        telemetry.addLine("");
        telemetry.addLine("─ CONTROLS ──────────────────────────────────────");
        telemetry.addLine("  RB = Toggle intake  |  LB = Save log");
        telemetry.addLine("  A  = Toggle kickback mode");
        telemetry.addLine("  Y  = Reset log  |  B = Emergency stop");
        telemetry.addLine("  RT = Manual IN  |  LT = Manual OUT");
        telemetry.update();
    }

    private String shortName(ShootSequence.BallColor c) {
        switch (c) {
            case GREEN:  return "G";
            case PURPLE: return "P";
            default:     return "-";
        }
    }
}