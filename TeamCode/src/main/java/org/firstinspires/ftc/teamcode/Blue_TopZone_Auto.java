package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue_TopZone_Auto", group = "Robot")
public class Blue_TopZone_Auto extends OpMode {

    public Follower follower1;
    public DcMotor shooter;
    private int pathState;
    private Limelight3A limelight;
    private final Pose startPose = new Pose(18, 121.2, Math.toRadians(270));
    private final Pose shootingPose = new Pose(60,72, Math.toRadians(315));
    private final Pose endPose = new Pose(70,50, Math.toRadians(315));
    private PathChain firstPath;

    @Override
    public void init() {
        follower1 = Constants.createFollower(hardwareMap);
        follower1.setStartingPose(startPose);
        follower1.setMaxPower(0.5);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        firstPath = follower1.pathBuilder()
                .addPath(new BezierLine(startPose, shootingPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootingPose.getHeading(), 0.8)

                .addParametricCallback(0.4, this::shooterOn)
                .addParametricCallback(1, this::intakeOn)
                .addParametricCallback(1,() -> haltThyBot(2000))
                .addParametricCallback(1, this::intakeShooterOff)

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
                follower1.followPath(firstPath, true);
                pathState = 2;
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
        shooter.setPower(0.45);
        return null;
    }

    public Runnable intakeOn() {
        follower1.pausePathFollowing();
        sleep(2000);
        follower1.resumePathFollowing();
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
