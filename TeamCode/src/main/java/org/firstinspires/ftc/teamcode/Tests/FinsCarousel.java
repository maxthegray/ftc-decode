//package org.firstinspires.ftc.teamcode.Tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Robot;
//
//@TeleOp(name = "Carousel fins localization", group = "Testing")
//public class FinsCarousel extends OpMode {
//
//    private Robot r;
//
//    // Color detection threshold
//    private int threshold = 1800;
//
//    private enum BallColor { GREEN, PURPLE, EMPTY }
//
//    // Slot tracking - stores what color ball is in each slot
//    private BallColor[] slots = { BallColor.EMPTY, BallColor.EMPTY, BallColor.EMPTY };
//
//    // Target sequence (for reference/comparison if needed)
//    private BallColor[] targetSequence = { BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE };
//
//    // Shooting sequence state
//    private enum ShootingState { IDLE, MOVING, FLICKING, DONE }
//    private ShootingState shootingState = ShootingState.IDLE;
//    private BallColor[] shootingOrder = null;
//    private int sequenceIndex = 0;
//
//    private static final double POWER = 0.5;
//    private static final double SLOW_POWER = 0.05;
//    private static final int NUM_POSITIONS = 3;
//
//    // Hole tracking
//    private boolean wasInHole = false;
//    private int holeCount = 1;
//
//    // Position tracking
//    private int currentPosition = 0;
//    private int targetPosition = 0;
//    private int direction = 0; // 1 = forward, -1 = backward, 0 = stopped
//
//    // Slowdown tracking
//    private boolean approachingAlignment = false;
//
//    // Flicker
//    private static final double DOWN = 0;
//    private static final double UP = 0.4;
//    private static final double FLICK_DURATION = 0.2;
//    private ElapsedTime flickTimer = new ElapsedTime();
//    private boolean isFlicking = false;
//
//    @Override
//    public void init() {
//        r = new Robot(hardwareMap);
//        r.init();
//        updateTelemetry();
//    }
//
//
//    @Override
//    public void loop() {
//        trackHoles();
//        updateSlotColors();
//
//        if (shootingState == ShootingState.IDLE) {
//            // Manual control mode
//            handleCarouselInput();
//            updateCarousel();
//            handleFlickerInput();
//            updateFlicker();
//            handleShootingInput();
//        } else {
//            // Automatic shooting sequence mode
//            updateShootingSequence();
//        }
//
//        updateTelemetry();
//    }
//
//    // Color detection methods
//
//    private BallColor detectColor(int red, int green, int blue) {
//        if (blue >= threshold) {
//            return BallColor.PURPLE;
//        } else if (green >= threshold) {
//            return BallColor.GREEN;
//        } else {
//            return BallColor.EMPTY;
//        }
//    }
//
//    private void updateSlotColors() {
//        if (!isCarouselSettled()) {
//            return; // Only read when settled
//        }
//
//        // Calculate which slots are over which sensors
//        int leftSensorSlot = (currentPosition + 2) % NUM_POSITIONS;
//        int rightSensorSlot = (currentPosition + 1) % NUM_POSITIONS;
//
//        // Read and store colors from each sensor
//        slots[leftSensorSlot] = detectColor(
//                r.blColor.red(),
//                r.blColor.green(),
//                r.blColor.blue()
//        );
//
//        slots[rightSensorSlot] = detectColor(
//                r.brColor.red(),
//                r.brColor.green(),
//                r.brColor.blue()
//        );
//    }
//
//    /**
//     * Marks the slot at the kicking position as empty after a kick completes.
//     */
//    private void markKickedSlotEmpty() {
//        slots[currentPosition] = BallColor.EMPTY;
//    }
//
//    /**
//     * Returns a string representation of a BallColor for telemetry.
//     */
//    private String ballColorToString(BallColor color) {
//        switch (color) {
//            case GREEN: return "GREEN";
//            case PURPLE: return "PURPLE";
//            case EMPTY: return "EMPTY";
//            default: return "?";
//        }
//    }
//
//    // Fin/hole methods
//
//    private void trackHoles() {
//        boolean inHole = isFinInHole();
//
//        // Entering a hole
//        if (inHole && !wasInHole) {
//            holeCount++;
//
//            if (isAlignmentHole()) {
//                currentPosition = (currentPosition + direction + NUM_POSITIONS) % NUM_POSITIONS;
//                approachingAlignment = false; // We've arrived, reset flag
//            }
//        }
//
//        // Exiting a hole - check if we just left a non-alignment hole
//        if (!inHole && wasInHole) {
//            if (holeCount % 2 == 1) {
//                // Just exited a non-alignment hole, alignment hole is next
//                approachingAlignment = true;
//            }
//        }
//
//        wasInHole = inHole;
//    }
//
//    private boolean isFinInHole() {
//        return !r.leftLim.getState() || !r.rightLim.getState();
//    }
//
//    private boolean isAlignmentHole() {
//        return holeCount % 2 == 0;
//    }
//
//    private boolean isInAlignmentHole() {
//        return isFinInHole() && isAlignmentHole();
//    }
//
//    // Carousel methods
//
//    private void handleCarouselInput() {
//        if (gamepad1.square) {
//            setTargetPosition(0);
//        } else if (gamepad1.triangle) {
//            setTargetPosition(1);
//        } else if (gamepad1.circle) {
//            setTargetPosition(2);
//        }
//    }
//
//    private void setTargetPosition(int newTarget) {
//        if (newTarget == targetPosition) {
//            return;
//        }
//
//        targetPosition = newTarget;
//        direction = calculateDirection();
//        holeCount = 0;
//        wasInHole = isFinInHole();
//        approachingAlignment = false; // Reset when starting new movement
//    }
//
//    private int calculateDirection() {
//        if (currentPosition == targetPosition) {
//            return 0;
//        }
//
//        int forwardDist = (targetPosition - currentPosition + 3) % 3;
//        int backwardDist = (currentPosition - targetPosition + 3) % 3;
//
//        return (forwardDist <= backwardDist) ? 1 : -1;
//    }
//
//    private void updateCarousel() {
//        if (isCarouselSettled() && targetPosition == currentPosition) {
//            r.carouselMotor.setPower(0);
//            direction = 0;
//        } else {
//            double power = approachingAlignment ? SLOW_POWER : POWER;
//            r.carouselMotor.setPower(power * direction);
//        }
//    }
//
//    private boolean isCarouselSettled() {
//        return currentPosition == targetPosition && isInAlignmentHole();
//    }
//
//    // Flicker methods
//
//    private void handleFlickerInput() {
//        if (!isCarouselSettled() || isFlicking) {
//            return;
//        }
//
//        if (gamepad1.dpad_up) {
//            isFlicking = true;
//            flickTimer.reset();
//        }
//    }
//
//    private void updateFlicker() {
//        if (isFlicking) {
//            if (flickTimer.seconds() < FLICK_DURATION) {
//                r.kicker.setPosition(UP);
//            } else {
//                r.kicker.setPosition(DOWN);
//                isFlicking = false;
//                // Ball was kicked - mark the slot at kicking position as empty
//                markKickedSlotEmpty();
//            }
//        } else {
//            r.kicker.setPosition(DOWN);
//        }
//    }
//
//    // Shooting sequence methods
//
//    /**
//     * Handles input to start the shooting sequence.
//     * Press dpad_down to start shooting with the targetSequence.
//     * Press dpad_down again while shooting to cancel.
//     */
//    private void handleShootingInput() {
//        if (gamepad1.dpad_down && isCarouselSettled()) {
//            startShootingSequence(targetSequence);
//        }
//    }
//
//    /**
//     * Starts an automatic shooting sequence with the given order.
//     * The carousel will rotate and shoot balls in the specified order.
//     * @param order Array of BallColors to shoot in order
//     */
//    public void startShootingSequence(BallColor[] order) {
//        if (order == null || order.length == 0) {
//            return;
//        }
//
//        shootingOrder = order;
//        sequenceIndex = 0;
//        queueNextShot();
//    }
//
//    /**
//     * Updates the shooting sequence state machine.
//     */
//    private void updateShootingSequence() {
//        // Allow canceling the sequence with dpad_down
//        if (gamepad1.dpad_down) {
//            stopShootingSequence();
//            return;
//        }
//
//        switch (shootingState) {
//            case MOVING:
//                updateCarousel();
//                if (isCarouselSettled()) {
//                    r.carouselMotor.setPower(0);
//                    direction = 0;
//                    shootingState = ShootingState.FLICKING;
//                    flickTimer.reset();
//                }
//                break;
//
//            case FLICKING:
//                if (flickTimer.seconds() < FLICK_DURATION) {
//                    r.kicker.setPosition(UP);
//                } else {
//                    r.kicker.setPosition(DOWN);
//                    markKickedSlotEmpty();
//                    sequenceIndex++;
//                    queueNextShot();
//                }
//                break;
//
//            case DONE:
//                r.carouselMotor.setPower(0);
//                // Sequence complete - can reset to IDLE if desired
//                break;
//
//            case IDLE:
//            default:
//                break;
//        }
//    }
//
//    /**
//     * Queues the next shot in the sequence.
//     * Finds a slot with the next required color and sets it as the target.
//     */
//    private void queueNextShot() {
//        if (shootingOrder == null || sequenceIndex >= shootingOrder.length) {
//            shootingState = ShootingState.DONE;
//            return;
//        }
//
//        BallColor nextColor = shootingOrder[sequenceIndex];
//        int slotWithBall = findSlotWithColor(nextColor);
//
//        if (slotWithBall == -1) {
//            // No ball of this color found, skip to next in sequence
//            sequenceIndex++;
//            queueNextShot();
//            return;
//        }
//
//        // If ball is already at kicking position, go straight to flicking
//        if (slotWithBall == currentPosition && isInAlignmentHole()) {
//            shootingState = ShootingState.FLICKING;
//            flickTimer.reset();
//        } else {
//            setTargetPosition(slotWithBall);
//            shootingState = ShootingState.MOVING;
//        }
//    }
//
//    /**
//     * Finds a slot containing a ball of the specified color.
//     * @param color The color to search for
//     * @return The slot index, or -1 if not found
//     */
//    private int findSlotWithColor(BallColor color) {
//        for (int i = 0; i < slots.length; i++) {
//            if (slots[i] == color) {
//                return i;
//            }
//        }
//        return -1;
//    }
//
//    /**
//     * Stops the shooting sequence and returns to manual control.
//     */
//    public void stopShootingSequence() {
//        shootingState = ShootingState.IDLE;
//        shootingOrder = null;
//        sequenceIndex = 0;
//        r.carouselMotor.setPower(0);
//        r.kicker.setPosition(DOWN);
//    }
//
//    /**
//     * Returns whether the shooting sequence is currently active.
//     */
//    public boolean isShootingSequenceActive() {
//        return shootingState != ShootingState.IDLE;
//    }
//
//    // Telemetry
//
//    private void updateTelemetry() {
//        telemetry.addData("Current Position", currentPosition);
//        telemetry.addData("Target Position", targetPosition);
//        telemetry.addData("Direction", direction);
//        telemetry.addData("Settled", isCarouselSettled() ? "Y" : "N");
//
//        telemetry.addData("Left Limit", r.getState(r.leftLim) ? "Y" : "N");
//
//        telemetry.addData("Hole Count", holeCount);
//        telemetry.addData("Approaching Alignment", approachingAlignment ? "Y" : "N");
//
//        telemetry.addData("Flicking", isFlicking ? "Y" : "N");
//
//        // Shooting sequence status
//        telemetry.addLine("--- Shooting Sequence ---");
//        telemetry.addData("Shooting State", shootingState);
//        if (shootingOrder != null) {
//            telemetry.addData("Progress", sequenceIndex + "/" + shootingOrder.length);
//        }
//
//        // Slot contents
//        telemetry.addLine("--- Slot Contents ---");
//        telemetry.addData("Slot 0", ballColorToString(slots[0]));
//        telemetry.addData("Slot 1", ballColorToString(slots[1]));
//        telemetry.addData("Slot 2", ballColorToString(slots[2]));
//
//        // Raw sensor readings for debugging
//        telemetry.addLine("--- Color Sensors ---");
//        telemetry.addData("Left (blColor)", "%d, %d, %d", r.blColor.red(), r.blColor.green(), r.blColor.blue());
//        telemetry.addData("Right (brColor)", "%d, %d, %d", r.brColor.red(), r.brColor.green(), r.brColor.blue());
//
//        // Show which slots are currently over sensors
//        if (isCarouselSettled()) {
//            int kickerSlot = currentPosition;
//            int leftSlot = (currentPosition + 2) % NUM_POSITIONS;
//            int rightSlot = (currentPosition + 1) % NUM_POSITIONS;
//            telemetry.addData("Kicker slot", kickerSlot);
//            telemetry.addData("Left sensor reading slot", leftSlot);
//            telemetry.addData("Right sensor reading slot", rightSlot);
//        }
//
//        telemetry.update();
//    }
//}