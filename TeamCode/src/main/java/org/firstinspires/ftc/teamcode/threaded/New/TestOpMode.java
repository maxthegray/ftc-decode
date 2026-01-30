package org.firstinspires.ftc.teamcode.threaded.New;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Sensor Test", group = "Test")
public class TestOpMode extends OpMode {

    private BotState state;
    private ControlHubI2C controlHub;
    private ExpansionHubI2C expansionHub;

    @Override
    public void init() {
        state = new BotState();
        controlHub = new ControlHubI2C(state, hardwareMap);
        expansionHub = new ExpansionHubI2C(state, hardwareMap);

        controlHub.start();
        expansionHub.start();

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addLine("=== OTOS ===");
        telemetry.addData("X", state.getOtosPosition().x);
        telemetry.addData("Y", state.getOtosPosition().y);
        telemetry.addData("H", state.getOtosPosition().h);

        telemetry.addLine("");
        telemetry.addLine("=== INTAKE ===");
        telemetry.addData("A", "a:%d b:%d g:%d",
                state.getAlphaA(BotState.POS_INTAKE),
                state.getBlueA(BotState.POS_INTAKE),
                state.getGreenA(BotState.POS_INTAKE));
        telemetry.addData("B", "a:%d b:%d g:%d",
                state.getAlphaB(BotState.POS_INTAKE),
                state.getBlueB(BotState.POS_INTAKE),
                state.getGreenB(BotState.POS_INTAKE));

        telemetry.addLine("");
        telemetry.addLine("=== BACK LEFT ===");
        telemetry.addData("A", "a:%d b:%d g:%d",
                state.getAlphaA(BotState.POS_BACK_LEFT),
                state.getBlueA(BotState.POS_BACK_LEFT),
                state.getGreenA(BotState.POS_BACK_LEFT));
        telemetry.addData("B", "a:%d b:%d g:%d",
                state.getAlphaB(BotState.POS_BACK_LEFT),
                state.getBlueB(BotState.POS_BACK_LEFT),
                state.getGreenB(BotState.POS_BACK_LEFT));

        telemetry.addLine("");
        telemetry.addLine("=== BACK RIGHT ===");
        telemetry.addData("A", "a:%d b:%d g:%d",
                state.getAlphaA(BotState.POS_BACK_RIGHT),
                state.getBlueA(BotState.POS_BACK_RIGHT),
                state.getGreenA(BotState.POS_BACK_RIGHT));
        telemetry.addData("B", "a:%d b:%d g:%d",
                state.getAlphaB(BotState.POS_BACK_RIGHT),
                state.getBlueB(BotState.POS_BACK_RIGHT),
                state.getGreenB(BotState.POS_BACK_RIGHT));

        telemetry.update();
    }

    @Override
    public void stop() {
        state.endThreads();
    }
}