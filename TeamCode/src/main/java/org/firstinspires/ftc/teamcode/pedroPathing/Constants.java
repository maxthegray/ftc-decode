package org.firstinspires.ftc.teamcode.pedroPathing;

import static java.lang.Math.PI;

import com.pedropathing.Drivetrain;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
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
            .mass(17.35)
            .forwardZeroPowerAcceleration(-64)
            .lateralZeroPowerAcceleration(-63)

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(.1, 0, .001, 0, .05))
            .useSecondaryDrivePIDF(true)
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.3, 0, .0001, 0, .3))

            .translationalPIDFCoefficients(new PIDFCoefficients(.05, 0, 0.01, 0.5))
            .useSecondaryTranslationalPIDF(true)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.6, 0, 0.03, 0.02))

            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0, 0.08, 0))
            .useSecondaryHeadingPIDF(true)
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1, 0, .09, .02))


// hi
            ;
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftRearMotorName("backLeftMotor")
            .leftFrontMotorName("frontLeftMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(49.5)
            .yVelocity(52.5)
            ;


    public static OTOSConstants localizerConstants = new OTOSConstants()
            .hardwareMapName("sensor_otos")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .linearScalar(1.15)
            .angularScalar(0.991)
            .offset(new SparkFunOTOS.Pose2D(0, 0, -PI/2));




    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 10, .7);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .OTOSLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
