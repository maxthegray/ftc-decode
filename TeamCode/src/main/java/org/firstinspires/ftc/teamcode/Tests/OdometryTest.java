package org.firstinspires.ftc.teamcode.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@TeleOp(name = "Odometry Test", group = "Test")
public class OdometryTest extends OpMode {

    private Follower   follower;
    private DcMotorEx  carouselMotor;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        carouselMotor = hardwareMap.get(DcMotorEx.class, "carousel_motor");
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addLine("Move the robot — pose will update live.");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleOpDrive();
    }

    @Override
    public void loop() {
        follower.update();

        Pose pose = follower.getPose();

        telemetry.addData("X",       "%.2f in", pose.getX());
        telemetry.addData("Y",       "%.2f in", pose.getY());
        telemetry.addData("Heading", "%.2f °",  Math.toDegrees(pose.getHeading()));
        telemetry.addLine("");
        telemetry.addData("Carousel Ticks", carouselMotor.getCurrentPosition());
        telemetry.update();
    }
}