package org.firstinspires.ftc.teamcode.commandbased.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandbased.subsystems.CarouselSubsystem;

public class KickCommand extends CommandBase {

    private final CarouselSubsystem carousel;
    private final ElapsedTime timer = new ElapsedTime();

    private static final double KICK_DURATION = 0.5;

    private enum Phase { WAITING, KICKING, RETRACTING }
    private Phase phase;

    public KickCommand(CarouselSubsystem carousel) {
        this.carousel = carousel;
        addRequirements(carousel);
    }

    @Override
    public void initialize() {
        if (carousel.isSettled()) {
            carousel.setKickerUp();
            timer.reset();
            phase = Phase.KICKING;
        } else {
            phase = Phase.WAITING;
        }
    }

    @Override
    public void execute() {
        switch (phase) {
            case WAITING:
                if (carousel.isSettled()) {
                    carousel.setKickerUp();
                    timer.reset();
                    phase = Phase.KICKING;
                }
                break;

            case KICKING:
                if (timer.seconds() >= KICK_DURATION) {
                    carousel.setKickerDown();
                    phase = Phase.RETRACTING;
                    timer.reset();
                }
                break;

            case RETRACTING:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return phase == Phase.RETRACTING && timer.seconds() >= 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        carousel.setKickerDown();
    }
}