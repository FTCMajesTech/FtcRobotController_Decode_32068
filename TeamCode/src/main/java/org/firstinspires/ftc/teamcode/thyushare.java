package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "amogus", group = "Robot")
public class thyushare extends OpMode {

    public Follower follower1;
    public DcMotor shooterMotorL;
    public DcMotor shooterMotorR;
    public DcMotor intakeMotor;
    public Servo intakeServo;
    private String pathState = "start";
    private final Pose startPose = new Pose(48, 0, Math.toRadians(90));
    private final Pose midPose = new Pose(48,48, Math.toRadians(45));
    private final Pose endPose = new Pose(0, 0, Math.toRadians(0));

    private PathChain firstPath, secondPath;





    @Override
    public void init() {
        follower1 = Constants.createFollower(hardwareMap);
        follower1.setStartingPose(startPose);
        follower1.setMaxPower(0.5);

        shooterMotorL = hardwareMap.get(DcMotor.class, "shooterMotorL");
        shooterMotorR = hardwareMap.get(DcMotor.class, "shooterMotorR");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        shooterMotorR.setDirection(FORWARD);
        shooterMotorL.setDirection(REVERSE);
        intakeMotor.setDirection(REVERSE);
        intakeServo.setDirection(Servo.Direction.REVERSE);

        firstPath = follower1.pathBuilder()
                .addPath(new BezierLine(startPose, midPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), midPose.getHeading(), 0.8)
                .addParametricCallback(0.5, this::skibidi)

                .addPath(new BezierLine(midPose, endPose))
                .setLinearHeadingInterpolation(midPose.getHeading(), endPose.getHeading(), 0.8)

                .addPath(new BezierCurve(endPose, startPose, new Pose(-48, 48,0), midPose))
                .setLinearHeadingInterpolation(endPose.getHeading(), midPose.getHeading())
                //.setHeadingInterpolation(HeadingInterpolator.facingPoint(startPose))
                //.setTangentHeadingInterpolation()
                .addParametricCallback(0.99, this::shoot)

                .build()
        ;
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        follower1.update();

        switch (pathState) {

            case "start":
                follower1.followPath(firstPath, true);
                pathState = "part one";
                break;

            case "part one":
                if(!follower1.isBusy()) {
                    pathState = "end program";
                }
                break;

            case "end program":
                telemetry.addData("donzo", "yippee");
                break;
        }

        telemetry.addData("x", follower1.getPose().getX());
        telemetry.addData("y", follower1.getPose().getY());
        telemetry.addData("heading", follower1.getPose().getHeading());
        telemetry.update();

    }
    public Runnable skibidi() {
        shooterMotorL.setPower(0.9);
        shooterMotorR.setPower(0.9);
        return null;
    }

    public Runnable shoot() {
        intakeServo.setPosition(1);
        return null;
    }
}