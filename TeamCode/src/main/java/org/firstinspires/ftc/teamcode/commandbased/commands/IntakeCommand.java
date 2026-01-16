package org.firstinspires.ftc.teamcode.commandbased.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandbased.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.commandbased.subsystems.CarouselSubsystem.BallColor;

public class IntakeCommand extends CommandBase {

    private final CarouselSubsystem carousel;

    private boolean ballDetectedThisCycle = false;

    public IntakeCommand(CarouselSubsystem carousel) {
        this.carousel = carousel;
        addRequirements(carousel);
    }

    @Override
    public void initialize() {
        ballDetectedThisCycle = false;
        carousel.runIntake();
    }

    @Override
    public void execute() {
        // Check if a ball just entered the intake position
        BallColor intakeContents = carousel.getIntakeContents();
        boolean hasBall = (intakeContents == BallColor.GREEN || intakeContents == BallColor.PURPLE);

        if (hasBall && !ballDetectedThisCycle) {
            // rotate to next empty position
            ballDetectedThisCycle = true;

            if (!carousel.isFull()) {
                carousel.rotateEmptyToIntake();
            }
        }

        // Reset detection flag when position becomes empty again
        if (!hasBall) {
            ballDetectedThisCycle = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        carousel.stopIntake();
    }

    @Override
    public boolean isFinished() {
        // Runs until interrupted or carousel is full
        return carousel.isFull();
    }
}