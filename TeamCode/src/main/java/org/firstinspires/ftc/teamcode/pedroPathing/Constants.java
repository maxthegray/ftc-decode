package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(1);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(98.1398454801304)
            .yVelocity(78.54551780880907)
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftRearMotorName("backLeftMotor")
            .leftFrontMotorName("frontLeftMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static OTOSConstants localizerConstants = new OTOSConstants()
            .hardwareMapName("odo")
            .linearUnit(DistanceUnit.INCH)
            .offset(new SparkFunOTOS.Pose2D(0,0,0))
            .angularScalar(.997)
            .linearScalar(-1.0)
            .angleUnit(AngleUnit.DEGREES);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .OTOSLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

}
