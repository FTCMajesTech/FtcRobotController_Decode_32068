package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous(name = "Red_Back_AutoShoot", group = "Robot")
public class Red_Back_AutoShoot extends OpMode {

    public Follower follower1;
    public DcMotorEx shooter;
    public DcMotor intake;
    public Servo shooterAngle;
    public Servo gate;
    public Servo pusher;
    private int pathState;
    private Limelight3A limelight;
    private IMU imu;
    private double distance;
    public double scale = 18398.87;
    private final Pose startPose = new Pose(84, 7, Math.toRadians(270));
    private final Pose startShoot = new Pose(84, 24, Math.toRadians(245));
    private final Pose shootingPose = new Pose(84,84, Math.toRadians(225));

    private final Pose GPPStart = new Pose(101,46.5, Math.toRadians(0));
    private final Pose GPPFinish = new Pose(116,46.5, Math.toRadians(0));

    private final Pose PGPStart = new Pose(101,68, Math.toRadians(0));
    private final Pose PGPFinish = new Pose(116,68, Math.toRadians(0));

    private final Pose PPGStart = new Pose(101,90, Math.toRadians(0));
    private final Pose PPGFinish = new Pose(116,90, Math.toRadians(0));

    private PathChain shoot1;






    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //intake = hardwareMap.get(DcMotor.class, "intake");
        //intake.setDirection(REVERSE);

        shooterAngle = hardwareMap.get(Servo.class, "shooterAngle");
        gate = hardwareMap.get(Servo.class, "gate");
        //pusher = hardwareMap.get(Servo.class, "pusher");

        shooterAngle.setPosition(1);
        gate.setPosition(.75);

        //connects limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        follower1 = Constants.createFollower(hardwareMap);
        follower1.setStartingPose(startPose);
        follower1.setMaxPower(0.5);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        shoot1 = follower1.pathBuilder()

                .addPath(new BezierLine(startPose, startShoot))
                .setLinearHeadingInterpolation(startPose.getHeading(), startShoot.getHeading(), 0.8)

                .build()
        ;
    }

    @Override
    public void start() {

        limelight.start();
    }

    @Override
    public void loop() {

        follower1.update();

        switch (pathState) {
            case 0:
                follower1.followPath(shoot1, true);
                pathState = 1;
                break;
            case 1:
                if (!follower1.isBusy()) {
                    pathState = 2;
                }
                break;
        }

        telemetry.addData("x", follower1.getPose().getX());
        telemetry.addData("y", follower1.getPose().getY());
        telemetry.addData("heading", follower1.getPose().getHeading());
        telemetry.update();

    }


    public Runnable shooterOn() {
        shooter.setPower(0.5);
        shooterAngle.setPosition(0.5);
        return null;
    }

    public Runnable shooterOff() {
        shooter.setPower(0);
        shooterAngle.setPosition(1);
        return null;
    }
    public Runnable haltThyBot(int tiempo) {
        follower1.pausePathFollowing();
        sleep(tiempo);
        follower1.resumePathFollowing();
        return null;
    }

    public Runnable intakeShooterOff() {
        shooter.setPower(0);
        return null;
    }
}