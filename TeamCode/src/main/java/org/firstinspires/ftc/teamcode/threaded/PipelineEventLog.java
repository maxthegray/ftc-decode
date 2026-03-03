package org.firstinspires.ftc.teamcode.threaded;

import android.os.Environment;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

/**
 * ═══════════════════════════════════════════════════════════════════════════
 *  PIPELINE EVENT LOG — zero-allocation timestamped event recorder
 * ───────────────────────────────────────────────────────────────────────────
 *  Pre-allocates a fixed-size ring buffer so MechanismThread can call
 *  record() without any object creation or GC pressure.
 *
 *  Usage:
 *    1. Create in OpMode:     PipelineEventLog log = new PipelineEventLog();
 *    2. Attach to MechThread: mechanismThread.setPipelineLog(log);
 *    3. After test:           log.writeToFile("pipeline_log");
 *
 *  All timestamps are in milliseconds relative to the first recorded event.
 * ═══════════════════════════════════════════════════════════════════════════
 */
public class PipelineEventLog {

    // ════════════════════════════════════════════════════════════════════════
    //  EVENT TYPES
    // ════════════════════════════════════════════════════════════════════════

    public enum Event {
        // Ramp sensor edges (detected in MechanismThread main loop)
        RAMP1_RISE,         // Lower ramp sensor (touch1) went high
        RAMP1_FALL,         // Lower ramp sensor (touch1) went low
        RAMP2_RISE,         // Upper ramp sensor (touch2) went high
        RAMP2_FALL,         // Upper ramp sensor (touch2) went low

        // Color sensor detection (in updateAutoIndex)
        COLOR_DETECTED,     // hasBallAtIntake() first returned true — debounce starts

        // Auto-index pipeline
        DEBOUNCE_DONE,      // Debounce timer expired — ball confirmed
        KICKBACK_START,     // Intake reversed (only when skipKickback=false)
        ROTATION_START,     // carousel.rotateSlots() called
        ROTATION_DONE,      // isMainMovementDone() returned true
        COOLDOWN_DONE,      // Post-move cooldown ticks reached zero — IDLE and ready

        // Loop timing (sampled, not every loop)
        LOOP_TIME,          // value field = loop duration in microseconds

        // Ramp voltage snapshot (for signal shape analysis)
        RAMP_VOLTAGE        // value field encodes both voltages (see below)
    }

    // ════════════════════════════════════════════════════════════════════════
    //  PRE-ALLOCATED STORAGE
    // ════════════════════════════════════════════════════════════════════════

    private static final int MAX_EVENTS = 4000;

    // Parallel arrays — no object allocation per event
    private final long[]  timestamps = new long[MAX_EVENTS];   // nanoTime
    private final int[]   eventTypes = new int[MAX_EVENTS];    // Event.ordinal()
    private final long[]  values     = new long[MAX_EVENTS];   // optional payload

    private volatile int count = 0;
    private long baseNanos = 0;
    private boolean baseSet = false;

    // ════════════════════════════════════════════════════════════════════════
    //  LOOP TIME SAMPLING — don't log every single loop, just enough to
    //  characterize the distribution
    // ════════════════════════════════════════════════════════════════════════

    private int loopSampleCounter = 0;
    private static final int LOOP_SAMPLE_INTERVAL = 50; // log every 50th loop

    // ════════════════════════════════════════════════════════════════════════
    //  RECORDING (called from MechanismThread — must be fast & lock-free)
    // ════════════════════════════════════════════════════════════════════════

    /**
     * Record an event with no payload.
     */
    public void record(Event event) {
        record(event, 0);
    }

    /**
     * Record an event with an associated value.
     */
    public void record(Event event, long value) {
        int idx = count;
        if (idx >= MAX_EVENTS) return; // silently drop if full

        long now = System.nanoTime();
        if (!baseSet) {
            baseNanos = now;
            baseSet = true;
        }

        timestamps[idx] = now;
        eventTypes[idx] = event.ordinal();
        values[idx]     = value;
        count = idx + 1;
    }

    /**
     * Record a loop iteration's duration. Only logs every LOOP_SAMPLE_INTERVAL
     * calls to avoid flooding the buffer.
     *
     * @param durationNanos elapsed time of one MechanismThread loop iteration
     */
    public void recordLoopTime(long durationNanos) {
        loopSampleCounter++;
        if (loopSampleCounter >= LOOP_SAMPLE_INTERVAL) {
            loopSampleCounter = 0;
            record(Event.LOOP_TIME, durationNanos / 1000); // store as microseconds
        }
    }

    /**
     * Record ramp sensor voltages. Encodes both into one long:
     *   upper 32 bits = ramp1 voltage * 10000 (as int)
     *   lower 32 bits = ramp2 voltage * 10000 (as int)
     *
     * Only call this periodically (e.g., every 10th loop) to avoid flooding.
     */
    public void recordRampVoltages(double ramp1V, double ramp2V) {
        long v1 = (long)(ramp1V * 10000);
        long v2 = (long)(ramp2V * 10000);
        long packed = (v1 << 32) | (v2 & 0xFFFFFFFFL);
        record(Event.RAMP_VOLTAGE, packed);
    }

    // ════════════════════════════════════════════════════════════════════════
    //  QUERY
    // ════════════════════════════════════════════════════════════════════════

    public int getCount() { return count; }

    public boolean isFull() { return count >= MAX_EVENTS; }

    // ════════════════════════════════════════════════════════════════════════
    //  FILE OUTPUT
    // ════════════════════════════════════════════════════════════════════════

    /**
     * Write all recorded events to a CSV file on the phone.
     * File is saved to /sdcard/FIRST/{filename}.csv
     *
     * @param filename base name (no extension)
     * @return the full path written, or an error message
     */
    public String writeToFile(String filename) {
        File dir = new File(Environment.getExternalStorageDirectory(), "FIRST");
        if (!dir.exists()) dir.mkdirs();

        File file = new File(dir, filename + ".csv");

        try (PrintWriter pw = new PrintWriter(new FileWriter(file))) {
            // Header
            pw.println("time_ms,event,value,notes");

            Event[] eventValues = Event.values();

            for (int i = 0; i < count; i++) {
                double timeMs = (timestamps[i] - baseNanos) / 1_000_000.0;
                Event evt = eventValues[eventTypes[i]];
                long val = values[i];

                String notes = "";
                if (evt == Event.RAMP_VOLTAGE) {
                    double r1 = (val >> 32) / 10000.0;
                    double r2 = (val & 0xFFFFFFFFL) / 10000.0;
                    notes = String.format("ramp1=%.4fV ramp2=%.4fV", r1, r2);
                } else if (evt == Event.LOOP_TIME) {
                    notes = val + "us";
                }

                pw.printf("%.3f,%s,%d,%s%n", timeMs, evt.name(), val, notes);
            }

            return file.getAbsolutePath();
        } catch (IOException e) {
            return "ERROR: " + e.getMessage();
        }
    }

    /**
     * Generate a human-readable summary with per-ball cycle breakdowns.
     * Finds groups of events between consecutive RAMP1_RISE events and
     * computes the time deltas between pipeline stages.
     */
    public String getSummary() {
        if (count == 0) return "No events recorded.";

        StringBuilder sb = new StringBuilder();
        Event[] eventValues = Event.values();

        // ── Overall stats ────────────────────────────────────────────────
        int ramp1Count = 0, ramp2Count = 0, colorCount = 0, rotationCount = 0;
        long loopTimeSum = 0;
        int loopTimeSamples = 0;
        long loopTimeMin = Long.MAX_VALUE, loopTimeMax = 0;

        for (int i = 0; i < count; i++) {
            Event evt = eventValues[eventTypes[i]];
            switch (evt) {
                case RAMP1_RISE:    ramp1Count++;    break;
                case RAMP2_RISE:    ramp2Count++;    break;
                case COLOR_DETECTED: colorCount++;   break;
                case ROTATION_DONE: rotationCount++; break;
                case LOOP_TIME:
                    long us = values[i];
                    loopTimeSum += us;
                    loopTimeSamples++;
                    if (us < loopTimeMin) loopTimeMin = us;
                    if (us > loopTimeMax) loopTimeMax = us;
                    break;
            }
        }

        sb.append("══ PIPELINE TIMING SUMMARY ══════════════════════════\n\n");
        sb.append(String.format("Total events:  %d / %d\n", count, MAX_EVENTS));
        sb.append(String.format("Balls detected (ramp1): %d\n", ramp1Count));
        sb.append(String.format("Ramp2 triggers:         %d\n", ramp2Count));
        sb.append(String.format("Color detections:       %d\n", colorCount));
        sb.append(String.format("Rotations completed:    %d\n\n", rotationCount));

        if (loopTimeSamples > 0) {
            sb.append(String.format("MechThread loop: avg=%dus  min=%dus  max=%dus  (n=%d)\n\n",
                    loopTimeSum / loopTimeSamples, loopTimeMin, loopTimeMax, loopTimeSamples));
        }

        // ── Per-ball cycle breakdown ─────────────────────────────────────
        // Walk through events and group by RAMP1_RISE boundaries
        sb.append("── PER-BALL CYCLE BREAKDOWN ─────────────────────────\n\n");

        int ballNumber = 0;
        int cycleStart = -1;

        for (int i = 0; i < count; i++) {
            Event evt = eventValues[eventTypes[i]];

            if (evt == Event.RAMP1_RISE) {
                // If we had a previous cycle, finalize it
                if (cycleStart >= 0) {
                    appendCycleBreakdown(sb, ballNumber, cycleStart, i - 1, eventValues);
                }
                ballNumber++;
                cycleStart = i;
            }
        }
        // Final cycle
        if (cycleStart >= 0) {
            appendCycleBreakdown(sb, ballNumber, cycleStart, count - 1, eventValues);
        }

        if (ballNumber == 0) {
            sb.append("  (No RAMP1_RISE events — feed balls through to generate data)\n");
        }

        return sb.toString();
    }

    private void appendCycleBreakdown(StringBuilder sb, int ballNum,
                                      int startIdx, int endIdx, Event[] eventValues) {
        sb.append(String.format("Ball #%d:\n", ballNum));

        long ramp1Time = timestamps[startIdx];

        // Find each event in this cycle window
        long ramp2Time    = -1;
        long colorTime    = -1;
        long debounceTime = -1;
        long kickbackTime = -1;
        long rotStartTime = -1;
        long rotDoneTime  = -1;
        long cooldownTime = -1;

        for (int i = startIdx + 1; i <= endIdx; i++) {
            Event evt = eventValues[eventTypes[i]];
            // Stop if we hit the next ball's ramp trigger
            if (evt == Event.RAMP1_RISE) break;

            switch (evt) {
                case RAMP2_RISE:     if (ramp2Time    < 0) ramp2Time    = timestamps[i]; break;
                case COLOR_DETECTED: if (colorTime    < 0) colorTime    = timestamps[i]; break;
                case DEBOUNCE_DONE:  if (debounceTime < 0) debounceTime = timestamps[i]; break;
                case KICKBACK_START: if (kickbackTime < 0) kickbackTime = timestamps[i]; break;
                case ROTATION_START: if (rotStartTime < 0) rotStartTime = timestamps[i]; break;
                case ROTATION_DONE:  if (rotDoneTime  < 0) rotDoneTime  = timestamps[i]; break;
                case COOLDOWN_DONE:  if (cooldownTime < 0) cooldownTime = timestamps[i]; break;
            }
        }

        // Print deltas
        appendDelta(sb, "  Ramp1 → Ramp2:       ", ramp1Time, ramp2Time);
        appendDelta(sb, "  Ramp1 → Color detect: ", ramp1Time, colorTime);
        appendDelta(sb, "  Color → Debounce done:", colorTime, debounceTime);
        if (kickbackTime > 0) {
            appendDelta(sb, "  Debounce → Kickback:  ", debounceTime, kickbackTime);
            appendDelta(sb, "  Kickback → Rot start: ", kickbackTime, rotStartTime);
        } else {
            appendDelta(sb, "  Debounce → Rot start: ", debounceTime, rotStartTime);
        }
        appendDelta(sb, "  Rot start → Rot done: ", rotStartTime, rotDoneTime);
        appendDelta(sb, "  Rot done → Cooldown:  ", rotDoneTime, cooldownTime);
        appendDelta(sb, "  ── TOTAL (Ramp1→Ready):", ramp1Time, cooldownTime);
        sb.append("\n");
    }

    private void appendDelta(StringBuilder sb, String label, long fromNanos, long toNanos) {
        if (fromNanos < 0 || toNanos < 0) {
            sb.append(label).append("  --\n");
        } else {
            double ms = (toNanos - fromNanos) / 1_000_000.0;
            sb.append(label).append(String.format("  %.1f ms\n", ms));
        }
    }
}