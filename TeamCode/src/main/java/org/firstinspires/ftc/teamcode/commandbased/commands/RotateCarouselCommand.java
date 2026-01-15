package org.firstinspires.ftc.teamcode.commandbased.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbased.subsystems.CarouselSubsystem;


public class RotateCarouselCommand extends CommandBase {

    public enum Direction { LEFT, RIGHT }

    private final CarouselSubsystem carousel;
    private final Direction direction;

    public RotateCarouselCommand(CarouselSubsystem carousel, Direction direction) {
        this.carousel = carousel;
        this.direction = direction;
        addRequirements(carousel);
    }

    @Override
    public void initialize() {
        if (direction == Direction.LEFT) {
            carousel.rotateOneStepBackward();
        } else {
            carousel.rotateOneStepForward();
        }
    }

    @Override
    public boolean isFinished() {
        return carousel.isSettled();
    }
}