package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp4", group="Robot")
public class TeleOp4 extends OpMode {
private Follower follower;
private Limelight3A limelight;
private IMU imu;
private double distance;
public double scale = 18398.87;
public DcMotorEx shooter;
public DcMotor intake, transfer;
public Servo shooterAngle, gate, pusher;
private double power = 0.25;
//how fast robot rotates when auto-aligning

//SHOOTER SETTINGS
private static final double SHOOTER_FAR_VELOCITY = 2400;    // ticks/sec for far shots
private static final double SHOOTER_CLOSE_VELOCITY = 1600;  // ticks/sec for close shots

// Clamp values for shooter distance mapping
private static final double FAR_DISTANCE = 2.5;   // meters
private static final double CLOSE_DISTANCE = 0.8; // meters

//AUTO-ALIGN SETTINGS
private static final double TX_TOLERANCE = 1.0;   // degrees, how close to target is "aligned"
private static final double ALIGN_KP = 0.015;     // proportional control factor for turning
private static final double MAX_ALIGN_POWER = 0.35; // max rotation speed during auto-align


    @Override
    public void init() {
        //creates drive follower
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(REVERSE);

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(FORWARD);

        shooterAngle = hardwareMap.get(Servo.class, "shooterAngle");
        gate = hardwareMap.get(Servo.class, "gate");
        pusher = hardwareMap.get(Servo.class, "pusher");

        shooterAngle.setPosition(1);

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
        //enables TeleOp driving and starts limelight
        follower.startTeleOpDrive();
        limelight.start();
        gate.setPosition(0);
        pusher.setPosition(0);

    }

    @Override
    public void loop() {
        follower.update();

        //updates drive system(left stick Y: forward/back, left stick X: strafe, right stick X: turn)
        follower.setTeleOpDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);

        //gets latest vision result
        LLResult result = limelight.getLatestResult();

        //
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

        if (gamepad1.dpad_up && result != null && result.isValid()) {

            //shooter stuff
            Pose3D botpose = result.getBotpose();
            if (gamepad1.dpad_up && result != null && result.isValid()) {
                // Use vertical angle Ty to estimate distance to target
                double ty = result.getTy();

                // Clamp Ty (vertical degrees to target) to reasonable values to prevent crazy shooter speeds
                double clampedTy = Math.max(-20, Math.min(-5, ty));  // adjust -20..-5 based on field/testing

                // Normalize Ty to 0..1 for interpolation
                double t = (clampedTy + 20) / 15; // -20 -> 0, -5 -> 1

                // Interpolate shooter velocity between far and close values
                double targetVelocity = SHOOTER_FAR_VELOCITY - t * (SHOOTER_FAR_VELOCITY - SHOOTER_CLOSE_VELOCITY);

                // Set shooter velocity
                shooter.setVelocity(targetVelocity);

            } else if (gamepad1.dpad_down) {
                shooter.setVelocity(0);
            }


        } else if (gamepad1.dpad_down) {
            shooter.setVelocity(0);
        }

        if (gamepad1.left_bumper) {
            autoAlign(result);
        }

        //shooter on
        if (gamepad1.a) {
            shooterAngle.setPosition(0.5);
            shooter.setPower(0.5);
        }

        if (gamepad1.b) {
            shooterAngle.setPosition(1);
            shooter.setPower(0);
        }

        if (gamepad1.x) {
            intake.setPower(0.75);
            transfer.setPower(0.75);
        }

        if (gamepad1.y) {
            intake.setPower(0);
            transfer.setPower(0);
        }

        if (gamepad1.dpad_left) {
            gate.setPosition(1);
            sleep(500);
            gate.setPosition(0);
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

        telemetry.update();
    }

    //helper movement methods

    // Auto-align robot toward target using Limelight Tx
    private void autoAlign(LLResult result) {
        if (result == null || !result.isValid()) {
            brake(); // stop rotation if no valid target
            return;
        }

        //horizontal offset
        double tx = result.getTx();

        // Stop if close enough to target
        if (Math.abs(tx) <= TX_TOLERANCE) {
            brake();
            return;
        }

        // Proportional control to determine rotation speed
        double turnPower = tx * ALIGN_KP;

        // Clamp turn speed so it doesn't spin too fast
        turnPower = Math.max(-MAX_ALIGN_POWER,
                Math.min(MAX_ALIGN_POWER, turnPower));

        follower.setTeleOpDrive(0, 0, turnPower);
    }

    public double getDistanceFromTage(double ta) {
        double distance = Math.pow(scale / ta,(1/1.8925));
        return distance;
    }

    public double getPowerFromDistance(double dist) {
        double amongus = 0.0007902299*dist + 0.4050294;
        return amongus;
    }


    private void brake() {
         //Sets all movement vectors to zero
        follower.setTeleOpDrive(0, 0, 0);
    }

}
