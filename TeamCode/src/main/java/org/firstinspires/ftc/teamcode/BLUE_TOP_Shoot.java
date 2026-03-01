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

@Autonomous(name = "BLUE_TOP_Shoot", group = "Robot")
public class BLUE_TOP_Shoot extends OpMode {
    // Variables
    public Follower follower1;
    public DcMotorEx shooter;
    public DcMotor intake, transfer, backTransfer;
    public Servo aim;
    public Servo gate;

    private double gateOpen = 0.43;
    private double gateClose = 0.25;
    private int shooterVelocity = 1000;
    private double setAim = 1.0;
    private double intakeOn = 0.75;
    private double transferOn = 1.0;

    private int pathState;
    private ElapsedTime timer = new ElapsedTime();

    // Poses
    private final Pose startPose = new Pose(15, 123.5, Math.toRadians(50));
    private final Pose shootingSpot = new Pose(48, 96, Math.toRadians(135));
    private final Pose endPose = new Pose(48, 125, Math.toRadians(180));

    // PathChains
    private PathChain initialShot,
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
                if (timer.seconds() > 2 && !follower1.isBusy()) {
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
                    intake.setPower(0);
                    transfer.setPower(0);
                    backTransfer.setPower(0);
                    follower1.followPath(endOfAuto, true);
                    nextState(3);
                }
                break;
            case 3:
                if (!follower1.isBusy()) {
                    nextState(4);
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
