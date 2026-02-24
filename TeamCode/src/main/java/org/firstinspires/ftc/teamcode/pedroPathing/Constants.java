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
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
//            .mass(17.35)
//            .forwardZeroPowerAcceleration(-76)
//            .lateralZeroPowerAcceleration(-68)
//
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0, .00006, .6, .1))
//            .useSecondaryDrivePIDF(true)
//            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0, .00003, .6, 0))
//
//            .translationalPIDFCoefficients(new PIDFCoefficients(.2, 0, 0.022, 0))
//            .useSecondaryTranslationalPIDF(true)
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0.025, 0.01))
//
//
//            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.1, 0))
//            .useSecondaryHeadingPIDF(true)
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1, 0, .09, .02))
//
//            .centripetalScaling(0.00009)


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
            ;


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5.372)
            .strafePodX(-6.441)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("sensor_otos")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
