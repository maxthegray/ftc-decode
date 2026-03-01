package org.firstinspires.ftc.teamcode.threaded.Old;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
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

    public void setIntakeRequest(IntakeRequest req) {
        this.intakeRequest = req;
    }

    // --- Hardware State Machine ---
    //
    //  IDLE ──► INTAKE_REVERSING ──► CAROUSEL_MOVING ──► IDLE
    //    │                                                 ▲
    //    ├──► CAROUSEL_MOVING ─────────────────────────────┤  (manual rotate / nudge)
    //    │                                                 │
    //    ├──► KICKER_UP ──► KICKER_DOWN ──────────────────┤  (manual kick)
    //    │                                                 │
    //    └──► SHOOT_WAIT_SHOOTER ──► SHOOT_ROTATING ──►   │  (single OR multi-shot)
    //         (or skip rotate)       SHOOT_KICKER_UP      │
    //                                 ──► SHOOT_KICKER_DOWN┘
    //                                      (loops back for next shot, or → IDLE)
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
    private static final long INTAKE_DEBOUNCE_MS = 75;

    // Shoot plan (works for single shots and full sequences)
    private int[] shotPlan = null;
    private int currentShotIndex = 0;
    private static final long SHOOTER_WAIT_TIMEOUT_MS = 1000;

    // Kicker safety delays
    private static final long KICKER_SAFETY_DELAY_MS = 200;  // Minimum time after commanding down
    private static final long KICKER_TIMEOUT_MS = 500;       // Maximum time to wait (failsafe)

    // Robot State
    private volatile ShootSequence.BallColor[] ballPositions = {
            ShootSequence.BallColor.EMPTY,
            ShootSequence.BallColor.EMPTY,
            ShootSequence.BallColor.EMPTY
    };
    private boolean autoIndexEnabled = true;
    private volatile boolean skipKickback = false;

    public void setSkipKickback(boolean skip) { this.skipKickback = skip; }

    // Sensor state
    private volatile SensorState sensorState = null;

    // Hold-to-nudge (bypasses command queue, overrides CAROUSEL_MOVING)
    private volatile int nudgeRequest = 0;
    private final ElapsedTime nudgeTimer = new ElapsedTime();
    private static final long NUDGE_INTERVAL_MS = 80;

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
            // 1. Update carousel controller (for Gaussian ramping)
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

            if (sensorState != null) {
                sensorState.setCarouselSpinning(!carousel.isMainMovementDone());
                boolean rampHit = rampSensor.getVoltage()  >= RAMP_SENSOR_THRESHOLD
                        || rampSensor2.getVoltage() >= RAMP_SENSOR_THRESHOLD;
                sensorState.setRampTriggered(rampHit);
            }
            // 2. Drain ALL queued commands
            processAllCommands();

            // 3. Run state machine
            switch (hardwareState) {

                // ---- IDLE: apply intake, check auto-index ----
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
                case SHOOT_WAIT_SHOOTER: {
                    boolean shooterReady = sensorState != null && sensorState.isShooterReady();

                    if (shooterReady) {
                        advanceToNextShot();
                    }
                    break;
                }

                case SHOOT_ROTATING:
                    if (carousel.isSettled()) {
                        kicker.up();
                        stateTimer.reset();
                        hardwareState = HardwareState.SHOOT_KICKER_UP;
                    }
                    break;

                case SHOOT_KICKER_UP:
                    if (kicker.isUp() || stateTimer.milliseconds() > 500) {
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
                        currentShotIndex++;
                        if (shotPlan == null || currentShotIndex >= shotPlan.length) {
                            // All shots fired
                            shotPlan = null;
                            currentShotIndex = 0;
                            hardwareState = HardwareState.IDLE;
                        } else {
                            // Next shot — shooter already spinning, go straight to rotate/kick
                            advanceToNextShot();
                        }
                    }
                    break;
                }
            }

            lights.update();

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                break;
            }
        }
    }

    // ==================== INTAKE ====================

    private void applyIntakeRequest() {
        switch (intakeRequest) {
            case IN:    intake.forward(); break;
            case OUT:   intake.reverse(); break;
            case STOP:  intake.stop();    break;
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
                    hardwareState = HardwareState.SHOOT_WAIT_SHOOTER;
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
                    hardwareState = HardwareState.SHOOT_WAIT_SHOOTER;
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
            kicker.up();
            stateTimer.reset();
            hardwareState = HardwareState.SHOOT_KICKER_UP;
        } else {
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
            return;
        }

        // Still waiting for the debounce window to expire.
        if (intakeDebounceTimer.milliseconds() < INTAKE_DEBOUNCE_MS) return;

        // Debounce passed — ball has been present for at least INTAKE_DEBOUNCE_MS.
        ballDebouncing = false;

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
            } else {
                intake.reverse();
                stateTimer.reset();
                hardwareState = HardwareState.INTAKE_REVERSING;
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
        if (isInShootState() && shotPlan != null && shotPlan.length > 1) {
            return String.format("%s | Shot %d/%d | Plan: %s",
                    hardwareState.name(), currentShotIndex + 1, shotPlan.length, planToString());
        }
        return hardwareState.name();
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