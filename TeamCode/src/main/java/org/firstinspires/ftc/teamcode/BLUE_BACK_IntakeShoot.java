package org.firstinspires.ftc.teamcode;

// Imports
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

@Autonomous(name = "BLUE_BACK_IntakeShoot", group = "Robot")
public class BLUE_BACK_IntakeShoot extends OpMode {
    // Variables
    public Follower follower1;
    public DcMotorEx shooter;
    public DcMotor intake, transfer, backTransfer;
    public Servo aim;
    public Servo gate;

    private double gateOpen = 0.43;
    private double gateClose = 0.25;
    private int shooterVelocity = 1550;
    private double setAim = 0.5;
    private double intakeOn = 0.75;
    private double transferOn = 1.0;

    private int pathState;
    private ElapsedTime timer = new ElapsedTime();

    // Poses
    private final Pose startPose = new Pose(57.5, 9, Math.toRadians(90));
    private final Pose shootingSpot = new Pose(57.5, 12, Math.toRadians(115));
    private final Pose closeArtifactStart = new Pose(57.5, 36, Math.toRadians(180));
    private final Pose closeArtifactCollect = new Pose(15, 36, Math.toRadians(180));
    private final Pose middleArtifactStart = new Pose(57.5, 60, Math.toRadians(180));
    private final Pose middleArtifactCollect = new Pose(15, 60, Math.toRadians(180));
    private final Pose endPose = new Pose(57.5, 36, Math.toRadians(180));

    // PathChains
    private PathChain initialShot,
            closeArtifactsStart,
            closeArtifactsCollect,
            closeArtifactsShoot,
            middleArtifactsStart,
            middleArtifactsCollect,
            middleArtifactsShoot,
            endOfAuto;

    @Override
    public void init() {
        // Follower Configs
        follower1 = Constants.createFollower(hardwareMap);
        follower1.setStartingPose(startPose);
        follower1.setMaxPower(1);

        // Shooter Configs
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(FORWARD);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder once to clear old data
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Turn on the internal Velocity PID mode

        // *** PIDF TUNING ***

        // Access the internal PIDF coefficients
        // P = Proportional (How hard it tries to reach the speed)(If motor is taking too long to get to target value, increase, vice versa)
        // I = Integral (How much it corrects for long-term errors)
        // D = Derivative (How much it prevents overshooting)
        // F = Feedforward (The "base" power needed for this speed)(If current velocity is too higher then target velocity and maintain it, reduce, vice versa)
        shooter.setVelocityPIDFCoefficients(160.0, 0.2, 18.0, 14.2);


        aim = hardwareMap.get(Servo.class, "aim");
        aim.setPosition(setAim); // 1=high arc 0=low arc

        // Intake Configs
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(REVERSE);

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(FORWARD);
        backTransfer = hardwareMap.get(DcMotorEx.class, "backTransfer");
        backTransfer.setDirection(REVERSE);

        gate = hardwareMap.get(Servo.class, "gate");
        gate.setPosition(.25);

        // Paths
        initialShot = follower1.pathBuilder()
                // go to shooting spot and fire the preloaded artifacts
                .addPath(new BezierLine(startPose, shootingSpot))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootingSpot.getHeading(), 0.5)
                .build();

        closeArtifactsStart = follower1.pathBuilder()
                // go from first shot to collect first row of artifacts
                .addPath(new BezierLine(shootingSpot, closeArtifactStart))
                .setLinearHeadingInterpolation(shootingSpot.getHeading(), closeArtifactStart.getHeading(), 0.5)
                .build();
        closeArtifactsCollect = follower1.pathBuilder()
                .addPath(new BezierLine(closeArtifactStart,closeArtifactCollect))
                .setLinearHeadingInterpolation(closeArtifactStart.getHeading(), closeArtifactCollect.getHeading(), 0.5)
                .build();
        closeArtifactsShoot = follower1.pathBuilder()
                .addPath(new BezierLine(closeArtifactCollect,shootingSpot))
                .setLinearHeadingInterpolation(closeArtifactCollect.getHeading(), shootingSpot.getHeading(), 0.5)
                .build();

        middleArtifactsStart = follower1.pathBuilder()
                //second shot to middle artifacts
                .addPath(new BezierLine(shootingSpot, middleArtifactStart))
                .setLinearHeadingInterpolation(shootingSpot.getHeading(), middleArtifactStart.getHeading(), 0.5)
                .build();
        middleArtifactsCollect = follower1.pathBuilder()
                .addPath(new BezierLine(middleArtifactStart, middleArtifactCollect))
                .setLinearHeadingInterpolation(middleArtifactStart.getHeading(), middleArtifactCollect.getHeading(), 0.5)
                .build();
        middleArtifactsShoot = follower1.pathBuilder()
                .addPath(new BezierLine(middleArtifactCollect,shootingSpot))
                .setLinearHeadingInterpolation(middleArtifactCollect.getHeading(), shootingSpot.getHeading(), 0.5)
                .build();

        endOfAuto = follower1.pathBuilder()
                .addPath(new BezierLine(shootingSpot, endPose))
                .setLinearHeadingInterpolation(shootingSpot.getHeading(), endPose.getHeading(), 0.5)
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
                shooter.setVelocity(shooterVelocity);
                nextState(1);   // <-- THIS resets timer
                break;
            case 1:
                if (timer.seconds() > 3 && !follower1.isBusy()) {
                    gate.setPosition(gateOpen);
                    intake.setPower(intakeOn);
                    transfer.setPower(transferOn);
                    backTransfer.setPower(transferOn);
                    nextState(2);
                }
                break;
            case 2:
                if (timer.seconds() > 2) {
                    shooter.setVelocity(0);
                    gate.setPosition(gateClose);
                    follower1.followPath(closeArtifactsStart, true);
                    nextState(3);
                }
                break;
            case 3:
                if (!follower1.isBusy()) {
                    follower1.followPath(closeArtifactsCollect, true);
                    shooter.setVelocity(shooterVelocity);
                    nextState(4);
                }
                break;
            case 4:
                if (!follower1.isBusy()) {
                    follower1.followPath(closeArtifactsShoot, true);
                    nextState(5);
                }
                break;
            case 5:
                if (timer.seconds() > 4 && !follower1.isBusy()) {
                    gate.setPosition(gateOpen);
                    nextState(6);
                }
                break;
            case 6:
                if (timer.seconds() > 3) {
                    shooter.setVelocity(0);
                    gate.setPosition(gateClose);
                    follower1.followPath(middleArtifactsStart, true);
                    nextState(7);
                }
                break;
            case 7:
                if (!follower1.isBusy()) {
                    follower1.followPath(middleArtifactsCollect, true);
                    shooter.setVelocity(shooterVelocity);
                    nextState(8);
                }
                break;
            case 8:
                if (!follower1.isBusy()) {
                    follower1.followPath(middleArtifactsShoot, true);
                    nextState(9);
                }
                break;
            case 9:
                if (timer.seconds() > 4 && !follower1.isBusy()) {
                    gate.setPosition(gateOpen);
                    nextState(10);
                }
            case 10:
                if (timer.seconds() > 4) {
                    shooter.setVelocity(0);
                    gate.setPosition(gateClose);
                    intake.setPower(0);
                    transfer.setPower(0);
                    backTransfer.setPower(0);
                    follower1.followPath(endOfAuto, true);
                    nextState(11);
                }
                break;
            case 11:
                if (!follower1.isBusy()) {
                    nextState(12);
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
