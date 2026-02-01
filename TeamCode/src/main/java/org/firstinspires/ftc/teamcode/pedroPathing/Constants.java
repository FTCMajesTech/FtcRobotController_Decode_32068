package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

public class Constants {
    public static Pose FCDPose = new Pose (24, 24);

    public static void setLastPose(Pose pose) {
        FCDPose = pose;
    }

    public Pose getLastPose() {
        return FCDPose;
    }

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.4)
            .forwardZeroPowerAcceleration( -50.31831224151835)
            .lateralZeroPowerAcceleration(-56.67518423396857)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0.0, 0.0025, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0.0, 0.005, 0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.027, 0.0, 0.0003, 0.6, 0.01))
            .centripetalScaling(0.0005)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.7)
            .rightFrontMotorName("FRdrive")
            .rightRearMotorName("BRdrive")
            .leftRearMotorName("BLdrive")
            .leftFrontMotorName("FLdrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(45.57758817898007)
            .yVelocity(35.909147728146536)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(169)
            .strafePodX(208)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("stalker")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            ;

    public static PathConstraints pathConstraints = new PathConstraints(1, 2, 1.7, 1.5);

    public static Follower createFollower(HardwareMap hardwareMap) {

        List<DcMotorEx> driveMotors = hardwareMap.getAll(DcMotorEx.class);
        for (DcMotorEx motor : driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
