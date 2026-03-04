package org.firstinspires.ftc.teamcode.threaded;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.threaded.CarouselController;
import org.firstinspires.ftc.teamcode.threaded.IntakeController;
import org.firstinspires.ftc.teamcode.threaded.KickerController;
import org.firstinspires.ftc.teamcode.threaded.LightsController;
import org.firstinspires.ftc.teamcode.threaded.PipelineEventLog;
import org.firstinspires.ftc.teamcode.threaded.SensorState;
import org.firstinspires.ftc.teamcode.threaded.ShootSequence;

import java.util.concurrent.ConcurrentLinkedQueue;

public class MechanismThread extends Thread {

    private final CarouselController carousel;
    private final KickerController kicker;
    private final IntakeController intake;
    private final LightsController lights;
    private final AnalogInput rampSensor;
    private final AnalogInput rampSensor2;

    private static final double RAMP_SENSOR_THRESHOLD = SensorState.RAMP_SENSOR_THRESHOLD;

    private final ConcurrentLinkedQueue<Command> commandQueue = new ConcurrentLinkedQueue<>();
    private volatile boolean killThread = false;

    // --- Intake Request (volatile field – never queued) ---
    public enum IntakeRequest { STOP, IN, OUT }
    private volatile IntakeRequest intakeRequest = IntakeRequest.STOP;
    private volatile double intakePower = 1.0;

    public void setIntakeRequest(IntakeRequest req) {
        this.intakeRequest = req;
    }

    public void setIntakeRequest(IntakeRequest req, double power) {
        this.intakeRequest = req;
        this.intakePower = power;
    }

    // --- Hardware State Machine ---
    //
    //  IDLE ──► INTAKE_REVERSING ──► CAROUSEL_MOVING ──► IDLE
    //    │                                                 ▲
    //    ├──► CAROUSEL_MOVING ─────────────────────────────┤  (manual rotate / nudge)
    //    │                                                 │
    //    ├──► KICKER_UP ──► KICKER_DOWN ──────────────────┤  (manual kick)
    //    │                                                 │
    //    └──► SHOOT_ROTATING ──────────────────────────►   │  (rotation needed: carousel
    //         (waits for carousel AND shooter)              │   + shooter spin up in parallel)
    //         ──► SHOOT_KICKER_UP ──► SHOOT_KICKER_DOWN ──┤
    //                                                      │
    //    └──► SHOOT_WAIT_SHOOTER ─────────────────────►    │  (no rotation: wait shooter only)
    //         ──► SHOOT_KICKER_UP ──► SHOOT_KICKER_DOWN ──┘
    //
    private enum HardwareState {
        IDLE,

        // Auto-index states
        INTAKE_REVERSING,
        CAROUSEL_MOVING,

        // Manual kick states
        KICKER_UP,
        KICKER_DOWN,

        // Shoot states (handles both single and multi-shot sequences)
        SHOOT_WAIT_SHOOTER,
        SHOOT_ROTATING,
        SHOOT_KICKER_UP,
        SHOOT_KICKER_DOWN
    }

    private HardwareState hardwareState = HardwareState.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();

    // Auto-Index
    private int pendingRotation = 0;
    private boolean autoIndexMove = false;  // true when CAROUSEL_MOVING was triggered by auto-index
    private static final long KICKBACK_MS = 50;

    // Post-move cooldown — suppress auto-index after any carousel move so stale I2C
    // readings don't trigger a second index before the sensors reflect the new position.
    // I2C updates every 50ms; 6 ticks (~60ms) is enough for one fresh read.
    // Stalls get a much longer window so the driver can correct before a retry.
    private int postMoveCooldownTicks = 0;
    private static final int POST_MOVE_COOLDOWN_TICKS  = 6;   // ~60ms  — normal move
    private static final int POST_STALL_COOLDOWN_TICKS = 30;  // ~300ms — stall

    // Intake debounce — ball must be present continuously for this long before indexing.
    private final ElapsedTime intakeDebounceTimer = new ElapsedTime();
    private boolean ballDebouncing = false;
    private static final long INTAKE_DEBOUNCE_MS = 15;

    // Shoot plan (works for single shots and full sequences)
    private int[] shotPlan = null;
    private int currentShotIndex = 0;
    private static final long SHOOTER_WAIT_TIMEOUT_MS = 1000;

    // Kicker safety delays
    private static final long KICKER_SAFETY_DELAY_MS = 200;  // Minimum time after commanding down
    private static final long KICKER_TIMEOUT_MS = 500;       // Maximum time to wait (failsafe)

    // ── Shot timing diagnostics ──────────────────────────────────────────────
    // Records how long each phase of the last shot took (ms).
    // Read from TeleOp for telemetry — volatile so they're visible cross-thread.
    private final ElapsedTime shotPhaseTimer = new ElapsedTime();
    private volatile long lastRotateMs       = 0;
    private volatile long lastWaitShooterMs  = 0;  // extra wait AFTER carousel settled
    private volatile long lastKickerUpMs     = 0;
    private volatile long lastKickerDownMs   = 0;

    public long getLastRotateMs()      { return lastRotateMs; }
    public long getLastWaitShooterMs() { return lastWaitShooterMs; }
    public long getLastKickerUpMs()    { return lastKickerUpMs; }
    public long getLastKickerDownMs()  { return lastKickerDownMs; }

    // Robot State
    private volatile ShootSequence.BallColor[] ballPositions = {
            ShootSequence.BallColor.EMPTY,
            ShootSequence.BallColor.EMPTY,
            ShootSequence.BallColor.EMPTY
    };
    private boolean autoIndexEnabled = true;
    private volatile boolean skipKickback = false;

    public void setSkipKickback(boolean skip) { this.skipKickback = skip; }

    // Pipeline diagnostic logger (null = disabled, zero cost)
    private volatile PipelineEventLog pipelineLog = null;
    public void setPipelineLog(PipelineEventLog log) { this.pipelineLog = log; }

    // ════════════════════════════════════════════════════════════════════════
    //  INTAKE GATING (Option C)
    // ════════════════════════════════════════════════════════════════════════
    //
    //  When enabled, MechanismThread overrides the OpMode's intake request:
    //  if the ramp sensor detects a ball AND the carousel is NOT ready to
    //  accept it, the intake is stopped — holding the ball on the ramp.
    //
    //  This eliminates the blind spot where the OpMode's check
    //  (rampTriggered && !mechIdle) misses balls arriving during the
    //  debounce / I2C-latency / cooldown window.
    //
    //  "Not ready" means ANY of:
    //    - A ball is already at the intake slot (color sensor sees non-EMPTY)
    //    - Debounce is in progress
    //    - Carousel is rotating (INTAKE_REVERSING or CAROUSEL_MOVING)
    //    - Post-move cooldown hasn't expired
    //
    //  The OpMode can query isIntakeGated() to react (e.g., pause driving).
    // ════════════════════════════════════════════════════════════════════════

    private volatile boolean autoGateIntake = false;
    private volatile boolean intakeCurrentlyGated = false;

    /** Enable/disable automatic intake gating. Typically ON for auto, OFF for teleop. */
    public void setAutoGateIntake(boolean enabled) { this.autoGateIntake = enabled; }

    /**
     * True when MechanismThread is actively overriding intake IN → STOP
     * because a ball is on the ramp and the carousel isn't ready.
     * OpMode can use this to pause driving.
     */
    public boolean isIntakeGated() { return intakeCurrentlyGated; }

    /**
     * Is the carousel pipeline completely clear and ready to accept a new ball?
     * Returns false if there's any reason a new ball should NOT enter.
     */
    private boolean isSafeForNewBall() {
        return hardwareState == HardwareState.IDLE
                && !ballDebouncing
                && postMoveCooldownTicks == 0
                && !hasBallAtIntake();
    }

    // Sensor state
    private volatile SensorState sensorState = null;

    // Hold-to-nudge (bypasses command queue, overrides CAROUSEL_MOVING)
    private volatile int nudgeRequest = 0;
    private final ElapsedTime nudgeTimer = new ElapsedTime();
    private static final long NUDGE_INTERVAL_MS = 20;

    // Pipeline logging — ramp sensor edge detection + voltage sampling
    private boolean prevRamp1Hit = false;
    private boolean prevRamp2Hit = false;
    private int rampVoltageSampleCounter = 0;

    public MechanismThread(HardwareMap hardwareMap) {
        this.carousel = new CarouselController(hardwareMap);
        this.kicker = new KickerController(hardwareMap);
        this.intake = new IntakeController(hardwareMap);
        this.lights = new LightsController(hardwareMap);
        this.rampSensor  = hardwareMap.get(AnalogInput.class, "touch1");
        this.rampSensor2 = hardwareMap.get(AnalogInput.class, "touch2");
    }

    public void setSensorState(SensorState state) {
        this.sensorState = state;
    }

    @Override
    public void run() {
        while (!killThread) {
            long _loopStartNanos = System.nanoTime();  // pipeline timing

            // 1. Update carousel controller (for PID loop)
            carousel.update();

            // Hold-to-nudge: fires every NUDGE_INTERVAL_MS while the driver holds the button.
            // Allowed in IDLE or CAROUSEL_MOVING (overrides any micro-adjust).
            // Blocked during shoot/kick sequences to avoid mid-sequence interference.
            if (nudgeRequest != 0
                    && (hardwareState == HardwareState.IDLE
                    || hardwareState == HardwareState.CAROUSEL_MOVING)
                    && nudgeTimer.milliseconds() >= NUDGE_INTERVAL_MS) {
                carousel.nudge(nudgeRequest);
                autoIndexMove = false;          // use isSettled(), not isMainMovementDone()
                if (hardwareState == HardwareState.IDLE) {
                    hardwareState = HardwareState.CAROUSEL_MOVING;
                }
                nudgeTimer.reset();
            }

            // ── Read ramp sensors ONCE per loop ───────────────────────────
            double ramp1Voltage = rampSensor.getVoltage();
            double ramp2Voltage = rampSensor2.getVoltage();
            boolean ramp1Hit = ramp1Voltage >= RAMP_SENSOR_THRESHOLD;
            boolean ramp2Hit = ramp2Voltage >= RAMP_SENSOR_THRESHOLD;
            boolean rampHit  = ramp1Hit || ramp2Hit;

            if (sensorState != null) {
                sensorState.setCarouselSpinning(!carousel.isMainMovementDone());
                sensorState.setRampTriggered(rampHit);
            }

            // ── Pipeline logging: ramp sensor edges + voltage ─────────────
            if (pipelineLog != null) {
                if (ramp1Hit && !prevRamp1Hit) pipelineLog.record(PipelineEventLog.Event.RAMP1_RISE);
                if (!ramp1Hit && prevRamp1Hit) pipelineLog.record(PipelineEventLog.Event.RAMP1_FALL);
                if (ramp2Hit && !prevRamp2Hit) pipelineLog.record(PipelineEventLog.Event.RAMP2_RISE);
                if (!ramp2Hit && prevRamp2Hit) pipelineLog.record(PipelineEventLog.Event.RAMP2_FALL);

                prevRamp1Hit = ramp1Hit;
                prevRamp2Hit = ramp2Hit;

                // Sample voltages every ~10 loops for signal shape
                rampVoltageSampleCounter++;
                if (rampVoltageSampleCounter >= 10) {
                    rampVoltageSampleCounter = 0;
                    pipelineLog.recordRampVoltages(ramp1Voltage, ramp2Voltage);
                }
            }

            // ── Intake gating decision ────────────────────────────────────
            // Computed every loop (~10ms) so there's no blind spot.
            // Two conditions that gate:
            //   1. Ball on ramp + carousel not ready (existing)
            //   2. BOTH ramp sensors high = two balls on ramp (one ball
            //      can't span both sensors, so this is always a pileup)
            if (autoGateIntake) {
                boolean twoBallsOnRamp = ramp1Hit && ramp2Hit;
                intakeCurrentlyGated = twoBallsOnRamp || (rampHit && !isSafeForNewBall());
            } else {
                intakeCurrentlyGated = false;
            }

            // 2. Drain ALL queued commands
            processAllCommands();

            // 3. Run state machine
            switch (hardwareState) {

                // ---- IDLE: apply intake (with gating), check auto-index ----
                case IDLE:
                    applyIntakeRequest();
                    updateAutoIndex();
                    break;

                // ---- AUTO-INDEX: kickback then rotate ----
                case INTAKE_REVERSING:
                    if (stateTimer.milliseconds() >= KICKBACK_MS) {
                        applyIntakeRequest();
                        carousel.rotateSlots(pendingRotation);
                        autoIndexMove = true;
                        hardwareState = HardwareState.CAROUSEL_MOVING;
                        if (pipelineLog != null) pipelineLog.record(PipelineEventLog.Event.ROTATION_START);
                    }
                    break;

                case CAROUSEL_MOVING:
                    boolean done = autoIndexMove
                            ? carousel.isMainMovementDone()
                            : carousel.isSettled();
                    if (done) {
                        autoIndexMove = false;
                        postMoveCooldownTicks = carousel.isStalled()
                                ? POST_STALL_COOLDOWN_TICKS
                                : POST_MOVE_COOLDOWN_TICKS;
                        hardwareState = HardwareState.IDLE;
                        if (pipelineLog != null) pipelineLog.record(PipelineEventLog.Event.ROTATION_DONE);
                    }
                    break;

                // ---- MANUAL KICK ----
                case KICKER_UP:
                    if (kicker.isUp() || stateTimer.milliseconds() > 500) {
                        kicker.down();
                        stateTimer.reset();
                        hardwareState = HardwareState.KICKER_DOWN;
                    }
                    break;

                case KICKER_DOWN: {
                    boolean kickerPhysicallyDown = kicker.isDown();
                    boolean safetyDelayPassed = stateTimer.milliseconds() >= KICKER_SAFETY_DELAY_MS;
                    boolean timedOut = stateTimer.milliseconds() >= KICKER_TIMEOUT_MS;

                    if ((kickerPhysicallyDown && safetyDelayPassed) || timedOut) {
                        hardwareState = HardwareState.IDLE;
                    }
                    break;
                }

                // ---- SHOOT (single or sequence) ----
                // SHOOT_WAIT_SHOOTER is now only entered when rotation == 0
                // (ball already at intake, just waiting on shooter speed).
                case SHOOT_WAIT_SHOOTER: {
                    boolean shooterReady = sensorState != null && sensorState.isShooterReady();

                    if (shooterReady) {
                        lastWaitShooterMs = (long) shotPhaseTimer.milliseconds();
                        lastRotateMs = 0;  // no rotation was needed
                        shotPhaseTimer.reset();
                        kicker.up();
                        stateTimer.reset();
                        hardwareState = HardwareState.SHOOT_KICKER_UP;
                    }
                    break;
                }

                case SHOOT_ROTATING: {
                    boolean settled = carousel.isSettled();
                    boolean shooterReady = sensorState != null && sensorState.isShooterReady();

                    if (settled && shooterReady) {
                        // Both ready — record timing and kick
                        lastRotateMs = (long) shotPhaseTimer.milliseconds();
                        lastWaitShooterMs = 0;  // shooter was ready by the time carousel finished (or vice versa)
                        shotPhaseTimer.reset();
                        kicker.up();
                        stateTimer.reset();
                        hardwareState = HardwareState.SHOOT_KICKER_UP;
                    } else if (settled && !shooterReady) {
                        // Carousel done but shooter still spinning up — record rotate time,
                        // stay here and keep waiting (don't transition to a separate state).
                        // lastRotateMs will be overwritten with the total when both are ready,
                        // but we track the split for telemetry.
                    }
                    // If !settled, keep waiting for carousel (shooter spinning up in parallel)
                    break;
                }

                case SHOOT_KICKER_UP:
                    if (kicker.isUp() || stateTimer.milliseconds() > 500) {
                        lastKickerUpMs = (long) shotPhaseTimer.milliseconds();
                        shotPhaseTimer.reset();
                        kicker.down();
                        stateTimer.reset();
                        hardwareState = HardwareState.SHOOT_KICKER_DOWN;
                    }
                    break;

                case SHOOT_KICKER_DOWN: {
                    boolean kickerPhysicallyDown = kicker.isDown();
                    boolean safetyDelayPassed = stateTimer.milliseconds() >= KICKER_SAFETY_DELAY_MS;
                    boolean timedOut = stateTimer.milliseconds() >= KICKER_TIMEOUT_MS;

                    if ((kickerPhysicallyDown && safetyDelayPassed) || timedOut) {
                        lastKickerDownMs = (long) shotPhaseTimer.milliseconds();
                        currentShotIndex++;
                        if (shotPlan == null || currentShotIndex >= shotPlan.length) {
                            // All shots fired
                            shotPlan = null;
                            currentShotIndex = 0;
                            hardwareState = HardwareState.IDLE;
                        } else {
                            // Next shot — start rotating immediately
                            shotPhaseTimer.reset();
                            advanceToNextShot();
                        }
                    }
                    break;
                }
            }

            lights.update();

            // Pipeline logging: loop time
            if (pipelineLog != null) {
                pipelineLog.recordLoopTime(System.nanoTime() - _loopStartNanos);
            }

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                break;
            }
        }
    }

    // ==================== INTAKE ====================

    /**
     * Apply the OpMode's intake request, respecting gating.
     * When gating is active and the OpMode wants IN, we override to STOP
     * so the ball parks on the ramp until the carousel is ready.
     */
    private void applyIntakeRequest() {
        if (intakeCurrentlyGated && intakeRequest == IntakeRequest.IN) {
            intake.stop();
            return;
        }

        switch (intakeRequest) {
            case IN:    intake.forward(intakePower); break;
            case OUT:   intake.reverse(intakePower); break;
            case STOP:  intake.stop();               break;
        }
    }

    // ==================== COMMAND PROCESSING ====================

    private void processAllCommands() {
        Command cmd;
        while ((cmd = commandQueue.poll()) != null) {
            processCommand(cmd);
        }
    }

    private void processCommand(Command cmd) {
        switch (cmd.type) {

            case KICK:
                if (hardwareState == HardwareState.IDLE && carousel.isSettled()) {
                    kicker.up();
                    stateTimer.reset();
                    hardwareState = HardwareState.KICKER_UP;
                }
                break;

            case ROTATE_LEFT:
                if (hardwareState == HardwareState.IDLE && carousel.isSettled()) {
                    carousel.rotateSlots(-1);
                    hardwareState = HardwareState.CAROUSEL_MOVING;
                }
                break;

            case ROTATE_RIGHT:
                if (hardwareState == HardwareState.IDLE && carousel.isSettled()) {
                    carousel.rotateSlots(1);
                    hardwareState = HardwareState.CAROUSEL_MOVING;
                }
                break;

            case NUDGE:
                // Superseded by setNudgeRequest() / hold-to-nudge.
                // Queued NUDGE commands are intentionally ignored.
                break;

            case SHOOT_SINGLE:
                if (hardwareState == HardwareState.IDLE
                        && carousel.isSettled()
                        && cmd.value instanceof ShootSequence.BallColor) {

                    ShootSequence.BallColor target = (ShootSequence.BallColor) cmd.value;
                    int rotation = ShootSequence.findRotationForColor(ballPositions, target);

                    if (rotation == Integer.MIN_VALUE) {
                        break;
                    }

                    intake.stop();
                    shotPlan = new int[] { rotation };
                    currentShotIndex = 0;
                    stateTimer.reset();
                    shotPhaseTimer.reset();
                    // Start rotating immediately — shooter spins up in parallel
                    advanceToNextShot();
                }
                break;

            case SHOOT_SEQUENCE:
                if (hardwareState == HardwareState.IDLE
                        && carousel.isSettled()
                        && cmd.value instanceof ShootSequence.BallColor[]) {

                    ShootSequence.BallColor[] targetOrder = (ShootSequence.BallColor[]) cmd.value;
                    int[] plan = ShootSequence.calculatePlan(ballPositions, targetOrder);

                    if (plan == null) {
                        break;
                    }

                    intake.stop();
                    shotPlan = plan;
                    currentShotIndex = 0;
                    stateTimer.reset();
                    shotPhaseTimer.reset();
                    // Start rotating immediately — shooter spins up in parallel
                    advanceToNextShot();
                }
                break;

            case ABORT_SEQUENCE:
                if (isInShootState()) {
                    kicker.down();
                    shotPlan = null;
                    currentShotIndex = 0;
                    hardwareState = HardwareState.IDLE;
                }
                break;

            case SET_AUTO_INDEX:
                this.autoIndexEnabled = (boolean) cmd.value;
                break;

            case SHOW_LIGHTS:
                lights.show(
                        ballColorToLight(ballPositions[0]),
                        ballColorToLight(ballPositions[1]),
                        ballColorToLight(ballPositions[2])
                );
                break;
        }
    }

    // ==================== SHOOT HELPERS ====================

    private void advanceToNextShot() {
        int rotation = shotPlan[currentShotIndex];
        if (rotation == 0) {
            // Ball already at intake — just need to wait for shooter speed
            boolean shooterReady = sensorState != null && sensorState.isShooterReady();
            if (shooterReady) {
                // Shooter already at speed — kick immediately
                lastWaitShooterMs = 0;
                lastRotateMs = 0;
                shotPhaseTimer.reset();
                kicker.up();
                stateTimer.reset();
                hardwareState = HardwareState.SHOOT_KICKER_UP;
            } else {
                hardwareState = HardwareState.SHOOT_WAIT_SHOOTER;
            }
        } else {
            // Start carousel rotation — shooter spins up in parallel
            carousel.rotateSlots(rotation);
            hardwareState = HardwareState.SHOOT_ROTATING;
        }
    }

    private boolean isInShootState() {
        return hardwareState == HardwareState.SHOOT_WAIT_SHOOTER
                || hardwareState == HardwareState.SHOOT_ROTATING
                || hardwareState == HardwareState.SHOOT_KICKER_UP
                || hardwareState == HardwareState.SHOOT_KICKER_DOWN;
    }

    // ==================== AUTO-INDEX ====================

    private void updateAutoIndex() {
        // After a stall, hold off briefly so we don't instantly retry on the same ball.
        if (postMoveCooldownTicks > 0) {
            postMoveCooldownTicks--;
            if (postMoveCooldownTicks == 0 && pipelineLog != null) {
                pipelineLog.record(PipelineEventLog.Event.COOLDOWN_DONE);
            }
            ballDebouncing = false;  // don't let stale data start the debounce timer
            return;
        }

        if (!autoIndexEnabled || isFull() || !kicker.isDown()) {
            ballDebouncing = false;
            return;
        }

        if (!hasBallAtIntake()) {
            // Ball gone — reset debounce so a future arrival starts fresh.
            ballDebouncing = false;
            return;
        }

        // Ball is present — start or continue the debounce window.
        if (!ballDebouncing) {
            ballDebouncing = true;
            intakeDebounceTimer.reset();
            if (pipelineLog != null) pipelineLog.record(PipelineEventLog.Event.COLOR_DETECTED);
            return;
        }

        // Still waiting for the debounce window to expire.
        if (intakeDebounceTimer.milliseconds() < INTAKE_DEBOUNCE_MS) return;

        // Debounce passed — ball has been present for at least INTAKE_DEBOUNCE_MS.
        ballDebouncing = false;
        if (pipelineLog != null) pipelineLog.record(PipelineEventLog.Event.DEBOUNCE_DONE);

        // Ball is at intake and there's an open slot — figure out where to rotate.
        // Read directly from sensorState for freshest possible back-slot data.
        ShootSequence.BallColor backLeft  = (sensorState != null)
                ? sensorState.getPositionColor(SensorState.POS_BACK_LEFT)
                : ballPositions[1];
        ShootSequence.BallColor backRight = (sensorState != null)
                ? sensorState.getPositionColor(SensorState.POS_BACK_RIGHT)
                : ballPositions[2];

        boolean leftEmpty  = (backLeft  == ShootSequence.BallColor.EMPTY);
        boolean rightEmpty = (backRight == ShootSequence.BallColor.EMPTY);

        int rotation = 0;
        if (leftEmpty && rightEmpty) {
            rotation = -1;                    // both empty → prefer back-right
        } else if (!leftEmpty && rightEmpty) {
            rotation = -1;                    // left full, right empty → back-right
        } else if (leftEmpty) {
            rotation = 1;                     // left empty, right full → back-left
        }
        // both full → isFull() should have caught this above

        if (rotation != 0) {
            pendingRotation = rotation;
            if (skipKickback) {
                carousel.rotateSlots(pendingRotation);
                autoIndexMove = true;
                hardwareState = HardwareState.CAROUSEL_MOVING;
                if (pipelineLog != null) pipelineLog.record(PipelineEventLog.Event.ROTATION_START);
            } else {
                intake.reverse();
                stateTimer.reset();
                hardwareState = HardwareState.INTAKE_REVERSING;
                if (pipelineLog != null) pipelineLog.record(PipelineEventLog.Event.KICKBACK_START);
            }
        }
    }

    // ==================== HELPERS ====================

    public void setBallPositions(ShootSequence.BallColor[] positions) {
        this.ballPositions = positions.clone();
    }

    private boolean hasBallAtIntake() {
        ShootSequence.BallColor c = ballPositions[0];
        return c != ShootSequence.BallColor.EMPTY;
    }

    private boolean isFull() {
        // Read directly from sensorState (same source as rotation decision) so both
        // agree on slot occupancy. Falls back to ballPositions if sensorState unavailable.
        if (sensorState != null) {
            int count = 0;
            for (int i = 0; i < 3; i++) {
                if (sensorState.getPositionColor(i) != ShootSequence.BallColor.EMPTY) count++;
            }
            return count >= 3;
        }
        int count = 0;
        for (ShootSequence.BallColor c : ballPositions) {
            if (c != ShootSequence.BallColor.EMPTY) count++;
        }
        return count >= 3;
    }

    private LightsController.Color ballColorToLight(ShootSequence.BallColor c) {
        switch (c) {
            case GREEN:  return LightsController.Color.GREEN;
            case PURPLE: return LightsController.Color.PURPLE;
            default:     return LightsController.Color.OFF;
        }
    }

    public String getStateDebug() {
        if (hardwareState == HardwareState.SHOOT_WAIT_SHOOTER) {
            boolean shooterReady = sensorState != null && sensorState.isShooterReady();
            return String.format("SHOOT_WAIT_SHOOTER | waiting %.0fms | ready=%b",
                    stateTimer.milliseconds(), shooterReady);
        }
        if (hardwareState == HardwareState.SHOOT_ROTATING) {
            boolean settled = carousel.isSettled();
            boolean shooterReady = sensorState != null && sensorState.isShooterReady();
            return String.format("SHOOT_ROTATING | %.0fms | car=%b shtr=%b",
                    shotPhaseTimer.milliseconds(), settled, shooterReady);
        }
        if (isInShootState() && shotPlan != null && shotPlan.length > 1) {
            return String.format("%s | Shot %d/%d | Plan: %s",
                    hardwareState.name(), currentShotIndex + 1, shotPlan.length, planToString());
        }
        String extra = intakeCurrentlyGated ? " | GATED" : "";
        return hardwareState.name() + extra;
    }

    private String planToString() {
        if (shotPlan == null) return "[]";
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < shotPlan.length; i++) {
            if (i == currentShotIndex) sb.append(">");
            sb.append(shotPlan[i]);
            if (i < shotPlan.length - 1) sb.append(", ");
        }
        sb.append("]");
        return sb.toString();
    }

    /** Set the active nudge direction while a button is held; call with 0 when released. */
    public void setNudgeRequest(int ticks) {
        this.nudgeRequest = ticks;
    }

    public void enqueueCommand(Command cmd) {
        commandQueue.add(cmd);
    }

    public int getCarouselCurrentTicks() {
        return carousel.getCurrentTicks();
    }

    public int getCarouselTargetTicks() {
        return carousel.getTargetTicks();
    }

    public boolean isCarouselSettled() {
        return carousel.isSettled();
    }

    public boolean isIdle() {
        return hardwareState == HardwareState.IDLE;
    }

    public void kill() {
        this.killThread = true;
    }

    // ==================== COMMAND CLASS ====================

    public static class Command {
        public enum Type {
            KICK,
            ROTATE_LEFT,
            ROTATE_RIGHT,
            NUDGE,              // value = Integer (signed tick count)
            SHOOT_SINGLE,       // value = BallColor
            SHOOT_SEQUENCE,     // value = BallColor[3] target order
            ABORT_SEQUENCE,     // abort running sequence
            SET_AUTO_INDEX,     // value = Boolean
            SHOW_LIGHTS         // show ball colors on lights
        }

        public final Type type;
        public final Object value;

        public Command(Type t) {
            this.type = t;
            this.value = null;
        }

        public Command(Type t, Object v) {
            this.type = t;
            this.value = v;
        }
    }
}