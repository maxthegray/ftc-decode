package org.firstinspires.ftc.teamcode.commandbased.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FeedBallCommand extends CommandBase {

    private final Telemetry telemetry;
    private boolean finished = false;

    public FeedBallCommand(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void initialize() {
        finished = false;
    }

    @Override
    public void execute() {

        telemetry.addLine("feeding ball");
        telemetry.update();

        // put servo code here
        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}