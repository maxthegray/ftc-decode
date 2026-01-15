package org.firstinspires.ftc.teamcode.commandbased.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbased.subsystems.CarouselSubsystem;


//kist rptate tp next empty slot (for auto indexing)
public class IndexCommand extends CommandBase {

    private final CarouselSubsystem carousel;

    public IndexCommand(CarouselSubsystem carousel) {
        this.carousel = carousel;
        addRequirements(carousel);
    }

    @Override
    public void initialize() {
        carousel.rotateEmptyToIntake();
    }

    @Override
    public boolean isFinished() {
        return carousel.isSettled();
    }
}