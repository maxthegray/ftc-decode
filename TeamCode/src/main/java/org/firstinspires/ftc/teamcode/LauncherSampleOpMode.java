package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

@TeleOp(name="LauncherTestOpMode", group="Testing")
public class LauncherSampleOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Example target sequence and carousel balls
        Integer[] targetSequence = {1, 2, 1};  // 1 = purple, 2 = green, found via apriltag
        Integer[] carouselBalls = {2, 1, 1};   // current carousel state, need to hook up color sensors to this

        // Create the Launcher instance
        Launcher launcher = new Launcher(
                Arrays.asList(targetSequence),
                Arrays.asList(carouselBalls),
                hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "servo_sample")
        );

        Localization localization = new Localization();

        telemetry.addData("Status", "initted");
        telemetry.addData("launcher", launcher);
        telemetry.addData("Lcarouselposition", launcher.LcarouselPosition);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Carousel main status", Arrays.toString(carouselBalls));
            telemetry.addData("Target main sequence", Arrays.toString(targetSequence));
            telemetry.addData("servo pos", launcher.servo.getPosition());

            telemetry.addData("Localization", localization.getPoseEstimate());

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
