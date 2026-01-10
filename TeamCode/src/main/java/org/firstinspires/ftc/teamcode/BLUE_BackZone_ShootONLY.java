package org.firstinspires.ftc.teamcode;

// Imports

import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BLUE_BackZone_ShootONLY", group = "Robot")
public class BLUE_BackZone_ShootONLY extends OpMode {

    // Variables
    public Follower follower1;
    public DcMotorEx shooter;
    public DcMotor intake, transfer;
    public Servo aim;
    public Servo gate;
    public Servo pusher;
    private int pathState;

    // Poses
    private final Pose startPose = new Pose(57.5, 9, Math.toRadians(90));
    private final Pose shootingSpot = new Pose(57.5, 12, Math.toRadians(120));
    private final Pose endPose = new Pose(57.5, 36, Math.toRadians(0));
    // PathChains
    private PathChain initialShot;

    @Override
    public void init() {
        // Follower Configs
        follower1 = Constants.createFollower(hardwareMap);
        follower1.setStartingPose(startPose);
        follower1.setMaxPower(0.75);

        // Shooter Configs
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        aim = hardwareMap.get(Servo.class, "aim");
        aim.setPosition(0.75); // 1=high arc 0=low arc

        // Intake Configs
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(REVERSE);

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(FORWARD);

        gate = hardwareMap.get(Servo.class, "gate");
        gate.setPosition(.32); // closed .32, open .1



        // Paths
        initialShot = follower1.pathBuilder()
                // go to shooting spot and fire the preloaded artifacts
                .addPath(new BezierLine(startPose, shootingSpot))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootingSpot.getHeading(), 0.5)

                // shooting artifacts
                .addParametricCallback(1, () -> haltThyBot(1500))
                .addParametricCallback(1, () -> gateControl(0.1))
                .addParametricCallback(1, () -> haltThyBot(1200))

                .addPath(new BezierLine(shootingSpot,endPose))
                .setLinearHeadingInterpolation(shootingSpot.getHeading(), endPose.getHeading(),0.5)

                .build();
    }


    @Override
    public void loop() {
        // Update Follower
        follower1.update();


        // Autonomous Path
        switch (pathState) {
            case 0:
                shooter.setPower(0.7);
                intake.setPower(0.9);
                transfer.setPower(0.75);
                follower1.followPath(initialShot, true);
                pathState = 1;
                break;
            case 1:
                if (!follower1.isBusy()) {
                    shooter.setPower(0);
                    intake.setPower(0);
                    transfer.setPower(0);
                    pathState = 2;
                }
                break;
        }

        telemetry.addData("x", follower1.getPose().getX());
        telemetry.addData("y", follower1.getPose().getY());
        telemetry.addData("heading", follower1.getPose().getHeading());
        telemetry.update();

    }

    // Runnables
    public Runnable gateControl(double position) {
        gate.setPosition(position);
        return null;
    }
    public Runnable haltThyBot(int tiempo) {
        follower1.pausePathFollowing();
        sleep(tiempo);
        follower1.resumePathFollowing();
        return null;
    }
}
