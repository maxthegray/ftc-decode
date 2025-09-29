package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;
import java.util.List;
@Disabled
@TeleOp(name="LauncherTestOpMode", group="Testing")
public class LauncherSampleOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Example target sequence and carousel balls
        List<Integer> exampleTargetSequence = Arrays.asList(0,0,0);  // 1 = purple, 2 = green, found via apriltag
        List<Integer> exampleCarouselBalls = Arrays.asList(2,1,1);   // current carousel state, need to hook up color sensors to this

        // Create the Launcher instance
        Launcher launcher = new Launcher(exampleTargetSequence, exampleCarouselBalls, hardwareMap, telemetry);

        //unified??
        UnifiedLocalization gps = new UnifiedLocalization(telemetry, hardwareMap, 0,0,0);

        telemetry.addData("Status", "initted");
        telemetry.addData("launcher", launcher);
        launcher.addTelemetry(telemetry);

        telemetry.update();

        boolean done = false;

        waitForStart();

        while (opModeIsActive()) {

            gps.updateAprilTag();

            launcher.step();

            telemetry.addData("Carousel main status", exampleCarouselBalls.toString());
            telemetry.addData("Target main sequence", exampleTargetSequence.toString());
            telemetry.addData("servo pos", launcher.servo.getPosition());

            gps.addTelemetry(   );
            telemetry.update();

            if (gamepad1.square) {
               telemetry.addData("Status", "busting!");

                telemetry.update();
                launcher.doBurst();
                telemetry.addData("Status", "bust complete");

                telemetry.update();



               sleep(2000); // wait 2 seconds so you can see the telemetry
           }


        }

    }

}
