package org.firstinspires.ftc.teamcode.commandbased.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandbased.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.CarouselSubsystem.BallColor;

/**
 * Shoots balls in a specified color sequence.
 * For each color: rotate that color to INTAKE, then kick.
 */
public class ShootSequenceCommand extends CommandBase {

    private final CarouselSubsystem carousel;
    private final BallColor[] sequence;

    private int sequenceIndex = 0;

    private enum Phase { FINDING_NEXT, MOVING, KICKING, DONE }
    private Phase phase;

    private static final double KICK_DURATION = 0.25;
    private static final double KICK_DELAY = 0.25;

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
                    kickTimer.reset();
                    phase = Phase.KICKING;
                }
                break;

            case KICKING:
                handleKicking();
                break;

            case DONE:
                break;
        }
    }

    private void findNextTarget() {
        while (sequenceIndex < sequence.length) {
            BallColor targetColor = sequence[sequenceIndex];

            // Check if we have this color
            if (carousel.hasColor(targetColor)) {
                // Rotate it to the kicker
                carousel.rotateToKicker(targetColor);
                phase = Phase.MOVING;
                return;
            }

            // Color not found, skip to next
            sequenceIndex++;
        }

        // No more targets
        phase = Phase.DONE;
    }

    private void handleKicking() {
        if (kickTimer.seconds() >= KICK_DELAY && kickTimer.seconds() < KICK_DELAY + KICK_DURATION) {
            carousel.setKickerUp();
        } else if (kickTimer.seconds() >= KICK_DELAY + KICK_DURATION) {
            carousel.setKickerDown();
            sequenceIndex++;
            phase = Phase.FINDING_NEXT;
        }
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