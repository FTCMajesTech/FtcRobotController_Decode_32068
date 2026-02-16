package org.firstinspires.ftc.teamcode;

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

@Autonomous(name = "RED_Top_IntakeShootV2", group = "Robot")
public class RED_Top_IntakeShoot_V2 extends OpMode {
    // Variables
    public Follower follower1;
    public DcMotorEx shooter;
    public DcMotor intake, transfer;
    public Servo aim;
    public Servo gate;
    public Servo pusher;

    private int pathState;

    // Poses
    private final Pose startPose = new Pose(125.5, 123.5, Math.toRadians(130));
    private final Pose shootingSpot = new Pose(96, 96, Math.toRadians(45));
    private final Pose closeArtifactStart = new Pose(86.5, 84, Math.toRadians(180));
    private final Pose closeArtifactCollect = new Pose(115, 84, Math.toRadians(180));
    private final Pose middleArtifactStart = new Pose(86.5, 60, Math.toRadians(180));
    private final Pose middleArtifactCollect = new Pose(120, 60, Math.toRadians(180));
    private final Pose endPose = new Pose(96, 60, Math.toRadians(90));
    // PathChains
    private PathChain initialShot, closeArtifacts, closeArtifacts2,middleArtifacts, middleArtifacts2, endOfAuto;



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
        gate.setPosition(.32); // closed .32, open .1

        // Paths
        initialShot = follower1.pathBuilder()
                // go to shooting spot and fire the preloaded artifacts
                .addPath(new BezierLine(startPose, shootingSpot))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootingSpot.getHeading(), 0.5)
                
                .build();

    }





    @Override
    public void loop() {

        // Update Follower
        follower1.update();

        // Autonomous Path
        switch (pathState) {
            case 0:
                shooter.setPower(0.4);
                follower1.followPath(initialShot, true);
                // shooting artifacts
                gate.setPosition(0.1);
                intake.setPower(0.9);
                transfer.setPower(1);
                pathState = 8;
                break;
            case 8:
                if (!follower1.isBusy()) {
                    shooter.setPower(0);
                    intake.setPower(0);
                    transfer.setPower(0);
                    pathState = 9;
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

    private Runnable intakeTransferOn() {
        intake.setPower(0.9);
        transfer.setPower(1);
        return null;
    }

    private Runnable intakeTransferOff() {
        intake.setPower(0);
        transfer.setPower(0);
        return null;
    }

    public Runnable haltThyBot(int tiempo) {
        follower1.pausePathFollowing();
        follower1.setTeleOpDrive(0, 0, 0);
        sleep(tiempo);
        follower1.resumePathFollowing();
        return null;
    }
}
