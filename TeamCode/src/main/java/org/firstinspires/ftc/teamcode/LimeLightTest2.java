package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="LimeLightTest2", group="Robot")
public class LimeLightTest2 extends OpMode {
private Limelight3A limelight;
private IMU imu;
private double distance;


    @Override
    public void init() {

        //connects limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        //telemetry updates every 11ms
        telemetry.setMsTransmissionInterval(11);

        //sets limelight pipeline(set of vision settings saved inside the Limelight) to 0
        limelight.pipelineSwitch(0);
        //pipeline 0= AprilTag detection
        //pipeline 1= color detection
        //pipeline 2= backup/testing
        //pipeline tells camera: what to look for(AprilTags, color blobs, etc.), how to process the image, and what data to output(Tx, Ty, pose, tags)

    }

    @Override
    public void start() {
        limelight.start();

    }

    @Override
    public void loop() {

        //gets latest vision result
        LLResult result = limelight.getLatestResult();

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            distance = getDistanceFromTage(llResult.getTa());
            telemetry.addData("Distance", distance);
            telemetry.addData("Target X", llResult.getTx());
            telemetry.addData("Target Area", llResult.getTa());
            telemetry.addData("BotPose", botPose.toString());
        }




        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                //horizontal error
                telemetry.addData("tx", result.getTx());
                //vertical error
                telemetry.addData("ty", result.getTy());
                //robot position
                telemetry.addData("Botpose", botpose.toString());
                //detected AprilTags
                telemetry.addData("april tag", result.getFiducialResults());
            }
        }
    }

    public double getDistanceFromTage(double ta) {
        double scale = 30665.95 ;
        double distance = (scale / ta);
        return distance;
    }
}
