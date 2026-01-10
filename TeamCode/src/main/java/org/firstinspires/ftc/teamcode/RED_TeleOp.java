package org.firstinspires.ftc.teamcode;

// Imports
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name="RED_TeleOp", group="Robot")
public class RED_TeleOp extends OpMode {
    // Variables
    private Follower follower;
    private Limelight3A limelight;
    private IMU imu;
    private double distance;
    public double scale = 18398.87;
    public DcMotorEx shooter;
    public DcMotor intake, transfer;
    public Servo aim, gate;

    // --- NEW CONSTANTS FOR AUTO-AIM ---
    // kP (Proportional) gain: Higher = faster turn, but might oscillate
    final double kP_AIM = 0.035;
    final double MIN_AIM_POWER = 0.05; // Minimum power to overcome friction
    // ----------------------------------

    @Override
    public void init() {
        // Create follower and update
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        // Shooter
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        aim = hardwareMap.get(Servo.class, "aim");
        aim.setPosition(1);

        // Intake and Transfer
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(REVERSE);

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(FORWARD);

        gate = hardwareMap.get(Servo.class, "gate");
        gate.setPosition(0.32);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0); // pipeline 0= AprilTag detection, pipeline 1= color detection, pipeline 2= backup/testing
    }

    @Override
    public void start() {
        follower.startTeleOpDrive();
        limelight.start();
    }

    @Override
    public void loop() {
        // Update follower
        follower.update();

        // Setting TeleOp drive
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        double rotate;
        double drive = 0;
        double strafe = 0;

        // Limelight
        LLResult result = limelight.getLatestResult();

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        // CHECK FOR AUTO-AIM TRIGGER (Holding Right Trigger)
        if (gamepad1.right_trigger > 0.1 && result != null && result.isValid()) {

            gate.setPosition(0.1);

            // 1. CALCULATE DISTANCE (Using your existing area-based method)
            distance = getDistanceFromTage(result.getTa());

            // 2. AUTO-ALIGN HEADING
            // tx is the horizontal offset. We want to rotate until tx is 0.
            double tx = result.getTx();
            rotate = -tx * kP_AIM;

            // Add small power if we aren't centered to overcome friction
            if (Math.abs(tx) > 0.5 && Math.abs(rotate) < MIN_AIM_POWER) {
                rotate = Math.signum(rotate) * MIN_AIM_POWER;
            }

            // 3. FINE-TUNE MOVEMENT (Slow down as robot gets closer)
            // If distance is 10, speed is 30%. If distance is 60, speed is 100%.
            double speedMultiplier = Range.scale(distance, 10, 60, 0.3, 1.0);
            drive *= speedMultiplier;
            strafe *= speedMultiplier;
            follower.setTeleOpDrive(drive, strafe, rotate);

            // 4. AUTOMATIC SHOOTER ADJUSTMENTS
            updateShooterHardware(distance);
        } else {
            shooter.setPower(0);
            aim.setPosition(1);
            gate.setPosition(0.32);
        }

        // Set drive controls
        //follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

        // int tagId = result.getFiducialResults().get(0).getFiducialId();

        // Telemetry
        if (result != null && result.isValid()) {
            telemetry.addData("Mode", "AUTO-AIM ACTIVE");
            telemetry.addData("Distance", distance);
            telemetry.addData("Target X", result.getTx());
        } else {
            telemetry.addData("Mode", "Manual Drive");
        }
        telemetry.update();

        // Manual Controls
        if (gamepad1.xWasPressed()) {
            shooter.setPower(0);
        }

        if (gamepad1.aWasPressed()) {
            intake.setPower(0.75);
            transfer.setPower(0.5);
        }

        if (gamepad1.bWasPressed()) {
            intake.setPower(0);
            transfer.setPower(0);
        }

        if (gamepad1.dpad_right) {
            gate.setPosition(0.1); // open
        }

        if (gamepad1.dpad_left) {
            gate.setPosition(0.32); // closed
        }

    }

    // Custom Methods
    public double getDistanceFromTage(double ta) {
        return Math.pow(scale / ta,(1/1.8925));
    }

    private void brake() {
        //Sets all movement vectors to zero
        follower.setTeleOpDrive(0, 0, 0);
    }

    // --- NEW METHOD: SHOOTER AUTOMATION ---
    private void updateShooterHardware(double dist) {
        // Example logic: adjust these numbers based on your testing!

        // SHOOTER SPEED: Faster when further away
        // Power = distance * slope + intercept
        double shooterPower = Range.clip((0.0007902299*dist + 0.4050294) + 0.5, 0.0, 1.0);
        shooter.setPower(shooterPower);

        // AIM SERVO: Higher angle when further away
        // Position = distance * slope + intercept
        double aimPosition = Range.clip((dist * 0.008) + 0.2, 1.0, 0.6);
        aim.setPosition(aimPosition);
    }
}
