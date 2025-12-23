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
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(7)
            .forwardZeroPowerAcceleration( -53.131905622567636)
            .lateralZeroPowerAcceleration(-77.75705337600189)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.052, 0.0, 0.0025, 0.024))
            .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0.0, 0.002, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.00055, 0.6, 0.01))
            .centripetalScaling(0.00053)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.67)
            .rightFrontMotorName("FrontRightDrive")
            .rightRearMotorName("BackRightDrive")
            .leftRearMotorName("BackLeftDrive")
            .leftFrontMotorName("FrontLeftDrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(27.921704630213462)
            .yVelocity(26.23818849578617)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-169)
            .strafePodX(-208)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("stalker")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            ;

    public static PathConstraints pathConstraints = new PathConstraints(1, 1, 0.5, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
