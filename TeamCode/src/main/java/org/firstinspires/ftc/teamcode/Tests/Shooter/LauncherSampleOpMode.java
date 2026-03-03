//package org.firstinspires.ftc.teamcode.Shooter;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.old.DriveTrain;
/// /import org.firstinspires.ftc.teamcode.Localization.UnifiedLocalization;
//
//import java.util.Arrays;
//import java.util.List;
//@Disabled
//@TeleOp(name="LauncherTestOpMode", group="Testing")
//public class LauncherSampleOpMode extends LinearOpMode {
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        // Example target sequence and carousel balls
//        List<Integer> exampleCarouselBalls = Arrays.asList(2,1,1);   // current carousel state, need to hook up color sensors to this
//
//        // Create the Launcher instance
//
//        //unified??
//        //UnifiedLocalization gps = new UnifiedLocalization(telemetry, hardwareMap);
//        Launcher launcher = new Launcher(exampleCarouselBalls, hardwareMap, telemetry, gamepad1);
//        DriveTrain driveTrain = new DriveTrain(hardwareMap, gamepad1, telemetry);
//
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            launcher.step();
//
//            driveTrain.drive();
//
//            telemetry.addData("Carousel main status", launcher.getBallsInCarousel());
//            telemetry.addData("servo pos", launcher.carousel.getPosition());
//
//            telemetry.update();
//
//            if (gamepad1.square) {
//                launcher.setBallsInCarousel(Arrays.asList(2,1,1));
//           }
//
//
//        }
//
//    }
//
//}
