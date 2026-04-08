package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.4)
            .forwardZeroPowerAcceleration(-37.3062)
            .lateralZeroPowerAcceleration(-52.3884)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.07, 0, 0.005, 0.025))
            .useSecondaryTranslationalPIDF(true)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.006, 0.015))

            .headingPIDFCoefficients(new PIDFCoefficients(1.55, 0, 0.1, 0.025))
            .useSecondaryHeadingPIDF(true)
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.07, 0.01))

//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02,0,0.0007,0,0.025))
//            .useSecondaryDrivePIDF(true)
//            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0, 0, 0, 0, 0))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.07, 0.08805436, 0.0008433916))

//            .centripetalScaling(0.00045)
            .centripetalScaling(0)
            ;


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(69.3306)
            .yVelocity(55.80826)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-68.538)
            .strafePodX(-132.605)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            ;

//    public static PathConstraints pathConstraints = new PathConstraints(0.975, 100, 2, 1);
    public static PathConstraints pathConstraints = new PathConstraints(0.97, 100, 2, 1);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
