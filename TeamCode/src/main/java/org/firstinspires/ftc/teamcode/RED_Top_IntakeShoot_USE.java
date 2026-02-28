package org.firstinspires.ftc.teamcode;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RED_Top_IntakeShoot(Use_This)", group = "Robot")
public class RED_Top_IntakeShoot_USE extends OpMode {
    // Variables
    public Follower follower1;
    public DcMotorEx shooter;
    public DcMotor intake, transfer, backTransfer;
    public Servo aim;
    public Servo gate;
    public Servo pusher;

    private int pathState;
    private ElapsedTime timer = new ElapsedTime();

    // Poses
    private final Pose startPose = new Pose(125.5, 123.5, Math.toRadians(130));
    private final Pose shootingSpot = new Pose(96, 96, Math.toRadians(45));
    private final Pose closeArtifactStart = new Pose(86.5, 84, Math.toRadians(180));
    private final Pose closeArtifactCollect = new Pose(115, 84, Math.toRadians(180));
    private final Pose middleArtifactStart = new Pose(86.5, 60, Math.toRadians(180));
    private final Pose middleArtifactCollect = new Pose(120, 60, Math.toRadians(180));
    private final Pose endPose = new Pose(96, 60, Math.toRadians(90));
    // PathChains
    private PathChain initialShot, closeArtifacts, closeArtifacts2, closeArtifacts3, middleArtifacts, middleArtifacts2, middleArtifacts3, endOfAuto;



    @Override
    public void init() {
        // Follower Configs
        follower1 = Constants.createFollower(hardwareMap);
        follower1.setStartingPose(startPose);
        follower1.setMaxPower(1);

        // Shooter Configs
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        aim = hardwareMap.get(Servo.class, "aim");
        aim.setPosition(1); // 1=high arc 0=low arc

        // Intake Configs
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(REVERSE);

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(FORWARD);
        backTransfer = hardwareMap.get(DcMotorEx.class, "backTransfer");
        backTransfer.setDirection(REVERSE);

        gate = hardwareMap.get(Servo.class, "gate");
        gate.setPosition(.25);  //0.32 is old close, new gate close is 0.25 , new gate open is 0.43, old gate open is 0.1

        // Paths
        initialShot = follower1.pathBuilder()
                // go to shooting spot and fire the preloaded artifacts
                .addPath(new BezierLine(startPose, shootingSpot))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootingSpot.getHeading(), 0.5)
                .build();

        closeArtifacts = follower1.pathBuilder()
                // go from first shot to collect first row of artifacts
                .addPath(new BezierLine(shootingSpot, closeArtifactStart))
                .setLinearHeadingInterpolation(shootingSpot.getHeading(), closeArtifactStart.getHeading(), 0.8)
                .build();
        closeArtifacts2 = follower1.pathBuilder()
                .addPath(new BezierLine(closeArtifactStart,closeArtifactCollect))
                .setLinearHeadingInterpolation(closeArtifactStart.getHeading(), closeArtifactCollect.getHeading(), 0.8)
                .build();
        closeArtifacts3 = follower1.pathBuilder()
                .addPath(new BezierLine(closeArtifactCollect,shootingSpot))
                .setLinearHeadingInterpolation(closeArtifactCollect.getHeading(), shootingSpot.getHeading(), 0.8)
                .build();

        middleArtifacts = follower1.pathBuilder()
                //second shot to middle artifacts
                .addPath(new BezierLine(shootingSpot, middleArtifactStart))
                .setLinearHeadingInterpolation(shootingSpot.getHeading(), middleArtifactStart.getHeading(), 0.8)
                .build();
        middleArtifacts2 = follower1.pathBuilder()
                .addPath(new BezierLine(middleArtifactStart, middleArtifactCollect))
                .setLinearHeadingInterpolation(middleArtifactStart.getHeading(), middleArtifactCollect.getHeading(), 0.8)
                .build();
        middleArtifacts3 = follower1.pathBuilder()
                .addPath(new BezierLine(middleArtifactCollect,shootingSpot))
                .setLinearHeadingInterpolation(middleArtifactCollect.getHeading(), shootingSpot.getHeading(), 0.8)
                .build();

        endOfAuto = follower1.pathBuilder()
                .addPath(new BezierLine(shootingSpot, endPose))
                .setLinearHeadingInterpolation(shootingSpot.getHeading(), endPose.getHeading(), 0.8)
                .build();
    }

    public void nextState(int newState) {
        pathState = newState;
        timer.reset();
    }


    @Override
    public void loop() {

        // Update Follower
        follower1.update();

        // Autonomous Path
        switch (pathState) {
            case 0:
                follower1.followPath(initialShot, true);
                shooter.setPower(0.6);
                pathState = 1;
                break;
            case 1:
                if (!follower1.isBusy()) {
                    intake.setPower(0.9);
                    transfer.setPower(1);
                    backTransfer.setPower(1);
                    gate.setPosition(.43);
                    nextState(2);    // <-- THIS resets timer
                }
                break;
            case 2:
                if (timer.seconds() > 3) {
                    shooter.setPower(0);   // stop shooter
                    follower1.followPath(closeArtifacts, true);
                    nextState(3);
                }
                break;
            case 3:
                if (!follower1.isBusy()) {
                    follower1.followPath(closeArtifacts2, true);
                    shooter.setPower(0.6);
                    intake.setPower(0.9);
                    transfer.setPower(1);
                    backTransfer.setPower(1);
                    gate.setPosition(.25);
                    nextState(4);
                }
                break;
            case 4:
                if (!follower1.isBusy()) {
                    follower1.followPath(closeArtifacts3, true);
                    gate.setPosition(.43);
                    nextState(5);
                }
                break;
            case 5:
                if (timer.seconds() > 3) {
                    shooter.setPower(0);
                    intake.setPower(0);
                    transfer.setPower(0);
                    backTransfer.setPower(0);
                    gate.setPosition(.25);
                    follower1.followPath(middleArtifacts, true);
                    nextState(6);
                }
                break;
            case 6:
                if (!follower1.isBusy()) {
                    follower1.followPath(middleArtifacts2, true);
                    shooter.setPower(0.6);
                    intake.setPower(0.9);
                    transfer.setPower(1);
                    backTransfer.setPower(1);
                    gate.setPosition(.25);
                    nextState(7);
                }
                break;
            case 7:
                if (!follower1.isBusy()) {
                    follower1.followPath(middleArtifacts3, true);
                    gate.setPosition(.43);
                    nextState(8);
                }
                break;
            case 8:
                if (timer.seconds() > 3) {
                    shooter.setPower(0);
                    intake.setPower(0);
                    transfer.setPower(0);
                    backTransfer.setPower(0);
                    gate.setPosition(.25);
                    follower1.followPath(endOfAuto, true);
                    nextState(9);
                }
                break;
            case 9:
                if (!follower1.isBusy()) {
                    nextState(10);
                }
                break;
        }

        telemetry.addData("x", follower1.getPose().getX());
        telemetry.addData("y", follower1.getPose().getY());
        telemetry.addData("heading", follower1.getPose().getHeading());
        telemetry.addData("State", pathState);
        telemetry.addData("Timer", timer.seconds());
        telemetry.update();
    }
}
