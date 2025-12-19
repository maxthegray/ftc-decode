// ============================================
// GoToPositionCommand.java
// ============================================

package org.firstinspires.ftc.teamcode.commandbased.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbased.subsystems.CarouselSubsystem;

public class GoToPositionCommand extends CommandBase {

    private final CarouselSubsystem carousel;
    private final int targetPosition;

    public GoToPositionCommand(CarouselSubsystem carousel, int position) {
        this.carousel = carousel;
        this.targetPosition = position;
        addRequirements(carousel);
    }

    @Override
    public void initialize() {
        carousel.goToPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return carousel.isSettled();
    }
}