package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;
import java.util.List;

@TeleOp(name="LauncherTestOpMode", group="Testing")
public class LauncherSampleOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Example target sequence and carousel balls
        List<Integer> targetSequence = Arrays.asList(0,0,0);  // 1 = purple, 2 = green, found via apriltag
        List<Integer> carouselBalls = Arrays.asList(2,1,1);   // current carousel state, need to hook up color sensors to this

        // Create the Launcher instance
        Launcher launcher = new Launcher(targetSequence, carouselBalls, hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "servo_sample"));
        Localization apriltags = new Localization(hardwareMap);

        telemetry.addData("Status", "initted");
        telemetry.addData("launcher", launcher);
        telemetry.addData("Lcarouselposition", launcher.LcarouselPosition);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            apriltags.update();

            if (apriltags.colorID==20) {
                targetSequence = Arrays.asList(2,1,1);
            } else if (apriltags.colorID==21) {
                targetSequence = Arrays.asList(1,2,1);
            } else if (apriltags.colorID==22) {
                targetSequence = Arrays.asList(1,1,2);
            }

            telemetry.addData("Carousel main status", carouselBalls.toString());
            telemetry.addData("Target main sequence", targetSequence.toString());
            telemetry.addData("servo pos", launcher.servo.getPosition());


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
