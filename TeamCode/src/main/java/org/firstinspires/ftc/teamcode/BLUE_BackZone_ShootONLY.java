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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BLUE_BackZone_ShootONLY", group = "Robot")
public class BLUE_BackZone_ShootONLY extends OpMode {

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
    private final Pose startPose = new Pose(57.5, 9, Math.toRadians(90));
    private final Pose shootingSpot = new Pose(57.5, 12, Math.toRadians(120));
    private final Pose endPose = new Pose(57.5, 36, Math.toRadians(90));
    // PathChains
    private PathChain initialShot, endOfAuto;


    @Override
    public void init() {
        // Follower Configs
        follower1 = Constants.createFollower(hardwareMap);
        follower1.setStartingPose(startPose);
        follower1.setMaxPower(0.67);

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

        gate = hardwareMap.get(Servo.class, "gate");
        gate.setPosition(.25); //0.32 is old close, new gate close is 0.25 , new gate open is 0.43, old gate open is 0.1



        // Paths
        initialShot = follower1.pathBuilder()
                // go to shooting spot and fire the preloaded artifacts
                .addPath(new BezierLine(startPose, shootingSpot))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootingSpot.getHeading(), 0.5)
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
        telemetry.update();

    }

}
