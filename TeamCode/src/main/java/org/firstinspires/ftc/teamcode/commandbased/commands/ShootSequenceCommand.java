// ============================================
// ShootSequenceCommand.java
// ============================================

package org.firstinspires.ftc.teamcode.commandbased.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandbased.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.CarouselSubsystem.BallColor;

public class ShootSequenceCommand extends CommandBase {

    private final CarouselSubsystem carousel;
    private final BallColor[] sequence;

    private int sequenceIndex = 0;

    private enum Phase { FINDING_NEXT, MOVING, KICKING, DONE }
    private Phase phase;

    private static final double KICK_DURATION = 0.5;
    private final ElapsedTime kickTimer = new ElapsedTime();

    public ShootSequenceCommand(CarouselSubsystem carousel, BallColor... sequence) {
        this.carousel = carousel;
        this.sequence = sequence;
        addRequirements(carousel);
    }

    @Override
    public void initialize() {
        sequenceIndex = 0;
        phase = Phase.FINDING_NEXT;
    }

    @Override
    public void execute() {
        switch (phase) {
            case FINDING_NEXT:
                findNextTarget();
                break;

            case MOVING:
                if (carousel.isSettled()) {
                    carousel.setKickerUp();
                    kickTimer.reset();
                    phase = Phase.KICKING;
                }
                break;

            case KICKING:
                if (kickTimer.seconds() >= KICK_DURATION) {
                    carousel.setKickerDown();
                    carousel.setSlotEmpty(carousel.getCurrentPosition());
                    sequenceIndex++;
                    phase = Phase.FINDING_NEXT;
                }
                break;

            case DONE:
                // Do nothing, waiting to finish
                break;
        }
    }

    private void findNextTarget() {
        // Skip colors that aren't in the carousel
        while (sequenceIndex < sequence.length) {
            BallColor targetColor = sequence[sequenceIndex];
            int slot = carousel.findSlotWithColor(targetColor);

            if (slot != -1) {
                // Found it, go there
                carousel.goToPosition(slot);
                phase = Phase.MOVING;
                return;
            }

            // Color not found, skip to next
            sequenceIndex++;
        }

        // No more targets
        phase = Phase.DONE;
    }

    @Override
    public boolean isFinished() {
        return phase == Phase.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        carousel.setKickerDown();
        carousel.stop();
    }
}