package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Launcher;

@TeleOp(name="LauncherTestOpMode", group="Testing")
public class LauncherSampleOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Example target sequence and carousel balls
        Integer[] targetSequence = {1, 2, 1};  // 1 = purple, 2 = green, found via apriltag
        Integer[] carouselBalls = {2, 1, 1};   // current carousel state, need to hook up color sensors to this

        // Create the Launcher instance
        Launcher launcher = new Launcher();

        telemetry.addData("Status", "initted");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
           if (gamepad1.a) {
               launcher.initBurst(targetSequence, carouselBalls);

               telemetry.addData("Status", "bust complete");
               telemetry.update();
               sleep(2000); // wait 2 seconds so you can see the telemetry
           }

        }
    }
}
