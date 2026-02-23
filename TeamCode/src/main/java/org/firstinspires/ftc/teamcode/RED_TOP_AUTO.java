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

@Autonomous(name = "Auto", group = "Robot")
public class RED_TOP_AUTO extends OpMode {
    // Variables
    public Follower follower1;

    private int pathState;

    // Poses
    private final Pose startPose = new Pose(86.5, 9, Math.toRadians(90));
    private final Pose shootingSpot = new Pose(86.5, 12, Math.toRadians(60));
    // PathChains
    private PathChain initialShot;



    @Override
    public void init() {
        // Follower Configs
        follower1 = Constants.createFollower(hardwareMap);
        follower1.setStartingPose(startPose);
        follower1.setMaxPower(0.4);

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
                follower1.followPath(initialShot, true);
                pathState = 6;
                break;

            case 6:
                if (!follower1.isBusy()) {
                    pathState = 7;
                }
                break;
        }

        telemetry.addData("x", follower1.getPose().getX());
        telemetry.addData("y", follower1.getPose().getY());
        telemetry.addData("heading", follower1.getPose().getHeading());
        telemetry.update();
    }
}
