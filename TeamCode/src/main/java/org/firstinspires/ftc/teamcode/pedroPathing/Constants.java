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
            .forwardZeroPowerAcceleration(-64.33828655584675)
            .lateralZeroPowerAcceleration(-64.03069803810496)

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0, 0, 0, 0, 0))
            .useSecondaryDrivePIDF(true)
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0, 0, 0, 0, 0))

            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.03))
            .useSecondaryTranslationalPIDF(true)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.19, 0, 0.009, 0.015))

            .headingPIDFCoefficients(new PIDFCoefficients(0.34, 0, 0.08, 0.2))
            .useSecondaryHeadingPIDF(true)
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.4, 0, 0.079, .02))



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
            .xVelocity(47.58399302565206)
            .yVelocity(46.72012929841289)
            ;


    public static OTOSConstants localizerConstants = new OTOSConstants()
            .hardwareMapName("sensor_otos")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .linearScalar(1.0627440238733985)
            .angularScalar(0.9903437854174533)
            .offset(new SparkFunOTOS.Pose2D(0, 0, -PI/2));




    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.2, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .OTOSLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
