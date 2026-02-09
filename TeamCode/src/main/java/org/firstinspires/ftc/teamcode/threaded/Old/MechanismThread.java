package org.firstinspires.ftc.teamcode.threaded.Old;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.ConcurrentLinkedQueue;

public class MechanismThread extends Thread {

    private final CarouselController carousel;
    private final KickerController kicker;
    private final IntakeController intake;
    private final LightsController lights;

    private final ConcurrentLinkedQueue<Command> commandQueue = new ConcurrentLinkedQueue<>();
    private volatile boolean killThread = false;

    // --- Intake Request (volatile field — never queued) ---
    public enum IntakeRequest { STOP, IN, OUT }
    private volatile IntakeRequest intakeRequest = IntakeRequest.STOP;

    public void setIntakeRequest(IntakeRequest req) {
        this.intakeRequest = req;
    }

    // --- Hardware State Machine ---
    //
    //  IDLE ──► INTAKE_REVERSING ──► CAROUSEL_MOVING ──► IDLE
    //    │                                                 ▲
    //    ├──► CAROUSEL_MOVING ─────────────────────────────┤  (manual rotate)
    //    │                                                 │
    //    ├──► KICKER_UP ──► KICKER_DOWN ───────────────────┤  (manual kick)
    //    │                                                 │
    //    └──► SHOOT_WAIT_SHOOTER ──► SHOOT_ROTATING ──►    │  (shoot-single: wait, rotate, kick)
    //         (or skip rotate)       SHOOT_KICKER_UP       │
    //                                ──► SHOOT_KICKER_DOWN─┘
    //
    private enum HardwareState {
        IDLE,

        // Auto-index states
        INTAKE_REVERSING,   // Kickback pulse (200ms reverse)
        CAROUSEL_MOVING,    // Waiting for carousel to settle (auto-index or manual rotate)

        // Manual kick states
        KICKER_UP,          // Kicker servo commanded up
        KICKER_DOWN,        // Waiting for kicker to physically return down

        // Shoot-single states (wait for shooter, rotate to color, then kick)
        SHOOT_WAIT_SHOOTER, // Waiting for shooter to reach target velocity
        SHOOT_ROTATING,     // Carousel rotating to bring target color to intake
        SHOOT_KICKER_UP,    // Kicker fired after rotation settled
        SHOOT_KICKER_DOWN   // Waiting for kicker to return after shot
    }

    private HardwareState hardwareState = HardwareState.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();

    // Auto-Index
    private int pendingRotation = 0;
    private static final long KICKBACK_MS = 200;

    // Shoot-single: rotation to apply after shooter is ready
    private int shootSingleRotation = 0;
    private static final long SHOOTER_WAIT_TIMEOUT_MS = 2000;

    // Safety timeout for kicker return (prevents permanent lockup if servo stalls)
    private static final long KICKER_DOWN_TIMEOUT_MS = 1000;

    // Robot State
    private volatile ShootSequence.BallColor[] ballPositions = {
            ShootSequence.BallColor.EMPTY,
            ShootSequence.BallColor.EMPTY,
            ShootSequence.BallColor.EMPTY
    };
    private boolean ballWasInIntake = false;
    private boolean autoIndexEnabled = true;

    // Sensor state (needed for shooter-ready checks)
    private volatile SensorState sensorState = null;

    public MechanismThread(HardwareMap hardwareMap) {
        this.carousel = new CarouselController(hardwareMap);
        this.kicker = new KickerController(hardwareMap);
        this.intake = new IntakeController(hardwareMap);
        this.lights = new LightsController(hardwareMap);
    }

    public void setSensorState(SensorState state) {
        this.sensorState = state;
    }

    @Override
    public void run() {
        while (!killThread) {
            // 1. Drain ALL queued commands
            processAllCommands();

            carousel.update();

            // 2. Run state machine
            switch (hardwareState) {

                // ---- IDLE: apply intake, check auto-index ----
                case IDLE:
                    applyIntakeRequest();
                    updateAutoIndex();
                    break;

                // ---- AUTO-INDEX: kickback then rotate ----
                case INTAKE_REVERSING:
                    if (stateTimer.milliseconds() >= KICKBACK_MS) {
                        intake.stop();
                        carousel.rotateSlots(pendingRotation);
                        hardwareState = HardwareState.CAROUSEL_MOVING;
                    }
                    break;

                case CAROUSEL_MOVING:
                    if (carousel.isSettled()) {
                        hardwareState = HardwareState.IDLE;
                    }
                    break;

                // ---- MANUAL KICK ----
                case KICKER_UP:
                    if (stateTimer.milliseconds() > 250) {
                        kicker.down();
                        stateTimer.reset();
                        hardwareState = HardwareState.KICKER_DOWN;
                    }
                    break;

                case KICKER_DOWN:
                    if (kicker.isDown() || stateTimer.milliseconds() > KICKER_DOWN_TIMEOUT_MS) {
                        hardwareState = HardwareState.IDLE;
                    }
                    break;

                // ---- SHOOT SINGLE: wait for shooter, rotate to color, then kick ----
                case SHOOT_WAIT_SHOOTER:
                    boolean shooterReady = sensorState != null && sensorState.isShooterReady();
                    boolean timedOut = stateTimer.milliseconds() >= SHOOTER_WAIT_TIMEOUT_MS;

                    if (shooterReady || timedOut) {
                        if (shootSingleRotation == 0) {
                            // Already at intake — kick immediately
                            kicker.up();
                            stateTimer.reset();
                            hardwareState = HardwareState.SHOOT_KICKER_UP;
                        } else {
                            // Rotate first, then kick
                            carousel.rotateSlots(shootSingleRotation);
                            hardwareState = HardwareState.SHOOT_ROTATING;
                        }
                    }
                    break;

                case SHOOT_ROTATING:
                    if (carousel.isSettled()) {
                        // Rotation done — now kick
                        kicker.up();
                        stateTimer.reset();
                        hardwareState = HardwareState.SHOOT_KICKER_UP;
                    }
                    break;

                case SHOOT_KICKER_UP:
                    if (stateTimer.milliseconds() > 250) {
                        kicker.down();
                        stateTimer.reset();
                        hardwareState = HardwareState.SHOOT_KICKER_DOWN;
                    }
                    break;

                case SHOOT_KICKER_DOWN:
                    if (kicker.isDown() || stateTimer.milliseconds() > KICKER_DOWN_TIMEOUT_MS) {
                        hardwareState = HardwareState.IDLE;
                    }
                    break;
            }

            lights.update();

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                break;
            }
        }
    }

    // ==================== INTAKE (volatile, not queued) ====================

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

            case SHOOT_SINGLE:
                if (hardwareState == HardwareState.IDLE
                        && carousel.isSettled()
                        && cmd.value instanceof ShootSequence.BallColor) {

                    ShootSequence.BallColor target = (ShootSequence.BallColor) cmd.value;
                    int rotation = findRotationForColor(ballPositions, target);

                    if (rotation == Integer.MIN_VALUE) {
                        // Color not found in carousel — do nothing
                        break;
                    }

                    intake.stop();  // Stop intake before any rotation/kick

                    // Store rotation and wait for shooter to reach speed
                    shootSingleRotation = rotation;
                    stateTimer.reset();
                    hardwareState = HardwareState.SHOOT_WAIT_SHOOTER;
                }
                break;

            case SET_AUTO_INDEX:
                this.autoIndexEnabled = (boolean) cmd.value;
                break;
        }
    }

    // ==================== AUTO-INDEX ====================

    private void updateAutoIndex() {
        if (!autoIndexEnabled || isFull() || !kicker.isDown()) {
            ballWasInIntake = hasBallAtIntake();
            return;
        }

        boolean hasBall = hasBallAtIntake();
        boolean ballJustArrived = hasBall && !ballWasInIntake;
        ballWasInIntake = hasBall;

        if (ballJustArrived) {
            ShootSequence.BallColor[] pos = ballPositions;
            int rotation = 0;

            // Directional logic (hardware-specific mapping)
            // Left (-1): slot 2 → intake | Right (1): slot 1 → intake
            if (pos[1] == ShootSequence.BallColor.EMPTY && pos[2] == ShootSequence.BallColor.EMPTY) {
                rotation = -1;  // Store first ball in Slot 1
            } else if (pos[1] != ShootSequence.BallColor.EMPTY && pos[2] == ShootSequence.BallColor.EMPTY) {
                rotation = -1;  // Bring Slot 2 to Intake
            } else if (pos[1] == ShootSequence.BallColor.EMPTY && pos[2] != ShootSequence.BallColor.EMPTY) {
                rotation = 1;   // Bring Slot 1 to Intake
            }

            if (rotation != 0) {
                pendingRotation = rotation;
                intake.reverse();       // Start kickback pulse
                stateTimer.reset();
                hardwareState = HardwareState.INTAKE_REVERSING;
            }
        }
    }

    // ==================== HELPERS ====================

    /**
     * Find which rotation brings the target color to the intake (position 0).
     * Returns: 0 if already at intake, 1 for right, -1 for left, MIN_VALUE if not found.
     */
    private int findRotationForColor(ShootSequence.BallColor[] positions, ShootSequence.BallColor color) {
        if (positions[0] == color) return 0;          // Already at intake
        if (positions[1] == color) return 1;           // Back-left → rotate right
        if (positions[2] == color) return -1;          // Back-right → rotate left
        return Integer.MIN_VALUE;                      // Not found
    }

    public void setBallPositions(ShootSequence.BallColor[] positions) {
        this.ballPositions = positions.clone();
    }

    private boolean hasBallAtIntake() {
        ShootSequence.BallColor c = ballPositions[0];
        return c == ShootSequence.BallColor.GREEN || c == ShootSequence.BallColor.PURPLE;
    }

    private boolean isFull() {
        int count = 0;
        for (ShootSequence.BallColor c : ballPositions) {
            if (c == ShootSequence.BallColor.GREEN || c == ShootSequence.BallColor.PURPLE) count++;
        }
        return count >= 3;
    }

    public String getStateDebug() {
        return hardwareState.name();
    }

    public void enqueueCommand(Command cmd) {
        commandQueue.add(cmd);
    }

    public void kill() {
        this.killThread = true;
    }

    // ==================== COMMAND CLASS ====================

    public static class Command {
        public enum Type {
            KICK,               // Manual single kick
            ROTATE_LEFT,        // Manual carousel rotate left 1 slot
            ROTATE_RIGHT,       // Manual carousel rotate right 1 slot
            SHOOT_SINGLE,       // Find color, rotate to intake, kick (value = BallColor)
            SET_AUTO_INDEX      // Enable/disable auto-indexing (value = Boolean)
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