package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.threaded.Old.ControlHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.Old.ExpansionHubI2CThread;
import org.firstinspires.ftc.teamcode.threaded.Old.MechanismThread;
import org.firstinspires.ftc.teamcode.threaded.Old.SensorState;
import org.firstinspires.ftc.teamcode.threaded.Old.ShootSequence;

/**
 * ═══════════════════════════════════════════════════════════════════════════
 *  INTAKE + CAROUSEL RAMP SENSOR TEST
 * ───────────────────────────────────────────────────────────────────────────
 *  No driving. Tests the full ball-feeding loop.
 *
 *  PHYSICS:
 *    The intake is a spinning wheel. The wheel grips and pushes balls.
 *    If the wheel stops, the ball stops with it — wherever it is on the ramp.
 *    The wheel MUST stay on the entire time to move a ball into the carousel.
 *
 *  LOGIC:
 *    - Intake runs continuously by default.
 *    - A ball on the ramp flows straight through into the carousel; the
 *      carousel auto-indexes to store it. No interruption needed.
 *    - If a SECOND ball reaches the ramp WHILE the carousel is still indexing
 *      the first, intake stops — the wheel parks the new ball in place.
 *    - Once the carousel finishes indexing, intake turns back on automatically
 *      and the parked ball continues in.
 *
 *  CONTROLS:
 *    Right Bumper (GP1) — Toggle intake manager ON / OFF
 *    Left Bumper  (GP1) — Emergency stop
 *    Right Trigger(GP1) — Manual intake IN  (bypasses manager while held)
 *    Left Trigger (GP1) — Manual intake OUT (unjam, bypasses manager while held)
 * ═══════════════════════════════════════════════════════════════════════════
 */
@TeleOp(name = "Intake Ramp Test", group = "Test")
public class IntakeRampTest extends OpMode {

    // ════════════════════════════════════════════════════════════════════════
    //  STATE MACHINE
    // ════════════════════════════════════════════════════════════════════════

    private enum IntakeState {
        /**
         * Normal running state. Intake wheel on.
         * Ball flows freely from ramp into carousel; carousel auto-indexes.
         * Only interrupts if the ramp sensor fires while the carousel is
         * already busy — in that case, transitions to HOLDING.
         */
        INTAKING,

        /**
         * A new ball reached the ramp while the carousel was still indexing.
         * Intake wheel stopped — ball is parked on the ramp by the still wheel.
         * Waits for the carousel to become idle, then returns to INTAKING so
         * the held ball can continue into the carousel.
         */
        HOLDING,

        /** Manager fully off. Intake stopped. */
        STOPPED
    }

    private IntakeState intakeState = IntakeState.STOPPED;

    // ════════════════════════════════════════════════════════════════════════
    //  HARDWARE / THREADS
    // ════════════════════════════════════════════════════════════════════════

    private MechanismThread       mechanismThread;
    private ControlHubI2CThread   controlHubI2C;
    private ExpansionHubI2CThread expansionHubI2C;
    private SensorState           sensorState;
    private AnalogInput           rampSensor;

    // ════════════════════════════════════════════════════════════════════════
    //  EDGE DETECTION
    // ════════════════════════════════════════════════════════════════════════

    private boolean prevRB = false;
    private boolean prevLB = false;

    // ════════════════════════════════════════════════════════════════════════
    //  LIFECYCLE
    // ════════════════════════════════════════════════════════════════════════

    @Override
    public void init() {
        sensorState = new SensorState(SensorState.Alliance.BLUE); // Alliance irrelevant here

        mechanismThread = new MechanismThread(hardwareMap);
        mechanismThread.setSensorState(sensorState);

        controlHubI2C   = new ControlHubI2CThread(sensorState, hardwareMap);
        expansionHubI2C = new ExpansionHubI2CThread(sensorState, hardwareMap);

        rampSensor = hardwareMap.get(AnalogInput.class, "touch1");

        // Enable auto-index so the carousel rotates when a ball enters
        mechanismThread.enqueueCommand(
                new MechanismThread.Command(MechanismThread.Command.Type.SET_AUTO_INDEX, true));

        telemetry.addLine("Intake Ramp Test — Ready");
        telemetry.addLine("RB = Start/Stop manager  |  LB = Emergency stop");
        telemetry.addLine("RT = Manual IN  |  LT = Manual OUT (unjam)");
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
        mechanismThread.setBallPositions(sensorState.getAllPositions());

        handleManualControls();
        runIntakeManager();
        updateTelemetry();
    }

    @Override
    public void stop() {
        mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
        mechanismThread.kill();
        sensorState.kill();
        try {
            mechanismThread.join(300);
            controlHubI2C.join(300);
            expansionHubI2C.join(300);
        } catch (InterruptedException ignored) {}
    }

    // ════════════════════════════════════════════════════════════════════════
    //  MANUAL CONTROLS
    // ════════════════════════════════════════════════════════════════════════

    private void handleManualControls() {
        // RB — Toggle manager on / off
        if (gamepad1.right_bumper && !prevRB) {
            if (intakeState == IntakeState.STOPPED) {
                intakeState = IntakeState.INTAKING;
            } else {
                intakeState = IntakeState.STOPPED;
                mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
            }
        }
        prevRB = gamepad1.right_bumper;

        // LB — Emergency stop
        if (gamepad1.left_bumper && !prevLB) {
            intakeState = IntakeState.STOPPED;
            mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
        }
        prevLB = gamepad1.left_bumper;
    }

    // ════════════════════════════════════════════════════════════════════════
    //  INTAKE MANAGER STATE MACHINE
    // ════════════════════════════════════════════════════════════════════════

    private void runIntakeManager() {
        // Manual triggers bypass the manager entirely while held
        if (gamepad1.right_trigger > 0.1) {
            mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);
            return;
        }
        if (gamepad1.left_trigger > 0.1) {
            mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.OUT);
            return;
        }

        boolean rampTriggered = sensorState.isRampTriggered();
        boolean mechIdle      = mechanismThread.isIdle();

        switch (intakeState) {

            // ── Intake on — ball flows freely ────────────────────────────────
            // Only pause if a new ball hits the ramp while the carousel is busy.
            // If the carousel is idle when the ramp fires, do nothing — ball
            // continues straight through and auto-index triggers on its own.
            case INTAKING:
                mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.IN);

                if (rampTriggered && !mechIdle) {
                    // New ball on ramp, carousel still busy — park it here
                    mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);
                    intakeState = IntakeState.HOLDING;
                }
                break;

            // ── Ball parked on ramp, waiting for carousel ─────────────────────
            // Intake wheel off — holds the ball still.
            // The moment the carousel finishes, turn intake back on so the
            // parked ball continues in and triggers a fresh auto-index.
            case HOLDING:
                mechanismThread.setIntakeRequest(MechanismThread.IntakeRequest.STOP);

                if (mechIdle) {
                    intakeState = IntakeState.INTAKING;
                    // Intake turns on at the top of INTAKING next frame
                }
                break;

            case STOPPED:
                break;
        }
    }

    // ════════════════════════════════════════════════════════════════════════
    //  TELEMETRY
    // ════════════════════════════════════════════════════════════════════════

    private void updateTelemetry() {
        double  rampVoltage   = rampSensor.getVoltage();
        boolean rampTriggered = sensorState.isRampTriggered();

        telemetry.addLine("══ INTAKE RAMP TEST ══════════════════════════════");
        telemetry.addLine("");

        telemetry.addData("Manager State", intakeState.name());
        telemetry.addData("Mechanism",
                mechanismThread.isIdle() ? "IDLE" : mechanismThread.getStateDebug());
        telemetry.addLine("");

        telemetry.addData("Ramp Sensor", "%.3fV  →  %s",
                rampVoltage, rampTriggered ? "TRIGGERED ◄" : "clear");
        telemetry.addLine("");

        ShootSequence.BallColor[] pos = sensorState.getAllPositions();
        telemetry.addData("Balls", "intake=%s  BL=%s  BR=%s",
                shortName(pos[0]), shortName(pos[1]), shortName(pos[2]));
        telemetry.addLine("");

        telemetry.addLine("─ CONTROLS ──────────────────────────────────────");
        telemetry.addLine("  RB = Toggle manager  |  LB = Emergency stop");
        telemetry.addLine("  RT = Manual IN  |  LT = Manual OUT (unjam)");
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