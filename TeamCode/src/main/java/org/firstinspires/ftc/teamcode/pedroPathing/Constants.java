package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
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
    public static MecanumConstants driveConstants = new MecanumConstants()
        .maxPower(1)
        .rightFrontMotorName("rf")
        .rightRearMotorName("rr")
        .leftRearMotorName("lr")
        .leftFrontMotorName("lf")
        .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
        .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
        .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
        .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
        .xVelocity(69.6326899490957)
        .yVelocity(57.420030969334405);
    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(9.6)
        .useSecondaryTranslationalPIDF(false)
        .useSecondaryHeadingPIDF(false)
        .useSecondaryDrivePIDF(false)
        .lateralZeroPowerAcceleration(-55.75335707400582)
        .forwardZeroPowerAcceleration(-34.55780728834356)
        .translationalPIDFCoefficients(new PIDFCoefficients(0.045, 0, 0.003, 0.021))
        .headingPIDFCoefficients(new PIDFCoefficients(0.6, 0.0, 0.005, 0.025))
        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0.0, 0.00001, 0.6, 0.01))
        .centripetalScaling(0.00045);


    public static PinpointConstants localizerConstants = new PinpointConstants()
        .forwardPodY(-3)
        .strafePodX(2.1)
        .distanceUnit(DistanceUnit.INCH)
        .hardwareMapName("pinpoint")
        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1.5,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
            .pinpointLocalizer(localizerConstants)
            .pathConstraints(pathConstraints)
            .mecanumDrivetrain(driveConstants)
            .build();
    }

}
