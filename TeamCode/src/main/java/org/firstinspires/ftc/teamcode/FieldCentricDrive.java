package org.firstinspires.ftc.teamcode;

// Imports
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
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

@TeleOp(name="FieldCentricDrive", group="Robot")
public class FieldCentricDrive extends OpMode {
    private Follower follower;
    private Limelight3A limelight;
    private IMU imu;
    private double distance;
    private double distanceFromGoal;
    public double scale = 18398.87;
    public DcMotorEx shooter;
    public DcMotor intake, transfer;
    public Servo aim, gate, pusher;
    // --- NEW CONSTANTS FOR AUTO-AIM ---
    // kP (Proportional) gain: Higher = faster turn, but might oscillate
    final double kP_AIM = 0.025;
    final double MIN_AIM_POWER = 0.05; // Minimum power to overcome friction
    private boolean centric = false;
    private boolean yPressedLastIteration = false;
    private double currentTargetAim = 0;
    private double currentTargetPower = 0;
    final double MAX_TICKS_PER_SECOND = 2600;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        follower.setStartingPose(Constants.FCDPose);

        // Shooter
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Access the internal PIDF coefficients
        // P = Proportional (How hard it tries to reach the speed)
        // I = Integral (How much it corrects for long-term errors)
        // D = Derivative (How much it prevents overshooting)
        // F = Feedforward (The "base" power needed for this speed)
        shooter.setVelocityPIDFCoefficients(30, 3, 0, 15);

        aim = hardwareMap.get(Servo.class, "aim");
        aim.setPosition(1);

        // Intake and Transfer
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(REVERSE);

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(FORWARD);

        gate = hardwareMap.get(Servo.class, "gate");
        gate.setPosition(0.32);

        pusher = hardwareMap.get(Servo.class, "pusher");
        pusher.setPosition(0.1); // 0.1=open  0.4=push up

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
        //Call this once per loop
        follower.update();

        boolean yIsPressed = gamepad1.y;

        // updates drive system(left stick Y: forward/back, left stick X: strafe, right stick X: turn)
        // Switching between field centric and robot centric
        if (yIsPressed && !yPressedLastIteration) {
            centric = !centric;
        }
        yPressedLastIteration = yIsPressed;

        //follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

        // Limelight
        LLResult result = limelight.getLatestResult();

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));



        // CHECK FOR AUTO-AIM TRIGGER (Holding Right/Left Trigger)
        // FAR Shooting
        if (gamepad1.right_trigger > 0.1 && result != null && result.isValid()) {

            gate.setPosition(0.1);

            // 1. CALCULATE DISTANCE (Using your existing area-based method)
            distance = getDistanceFromTage(result.getTa());


            double targetPower = (0.0008702083 * distance) + 0.3177145;
            double targetAimPos = 0.4682029 + (0.001781158 * distance) - (0.000005436021 * Math.pow(distance, 2));

            double targetVelocity = targetPower * MAX_TICKS_PER_SECOND;

            // 3. SAFETY: Clip the values to make sure they stay within safe values
            currentTargetAim = Range.clip(targetAimPos, 0.3, 1.0);
            currentTargetPower = Range.clip(targetPower, 0.0, 1.0);

            // 4. Apply to hardware
            aim.setPosition(currentTargetAim);
            shooter.setVelocity(targetVelocity);

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

        } else {
            shooter.setPower(0);
            aim.setPosition(1);
            gate.setPosition(0.32);
        }

        follower.setTeleOpDrive(drive, strafe, rotate, centric);

        // Manual Controls
        if (gamepad1.bWasPressed()) {
            shooter.setPower(0);
            aim.setPosition(1);
            gate.setPosition(0.32);
        }

        if (gamepad1.aWasPressed()) {
            intake.setPower(.75);
            transfer.setPower(0.6);
        }

        if (gamepad1.xWasPressed()) {
            intake.setPower(0);
            transfer.setPower(0);
        }

        // override for gate to open (closes automatically due to trigger else statement)
        if (gamepad1.dpad_left) {
            gate.setPosition(0.1);
        }

        // Telemetry
        if (result != null && result.isValid()) {
            distanceFromGoal = getDistanceFromTage(result.getTa());

            telemetry.addData("Mode", "AUTO-AIM ACTIVE");
            telemetry.addData("Distance", distanceFromGoal);
            telemetry.addData("Target X", result.getTx());
            telemetry.addData("Calc Aim Pos", "%.3f", currentTargetAim);
            telemetry.addData("Calc Power", "%.3f", currentTargetPower);

        } else {
            telemetry.addData("Mode", "Manual Drive");
        }
        telemetry.update();
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
    private void updateShooterHardware(double x) {

        // 10 sec to get shooter working correctly

        // Example logic: adjust these numbers based on your testing!

        // SHOOTER SPEED: Faster when further away
        // Power = distance * slope + intercept
        //double shooterPower = Range.clip((0.0007902299*dist) + 0.4050294, 0.0, 1.0);

        // AIM SERVO: Higher angle when further away
        // Position = distance * slope + intercept
        //double aimPosition = Range.clip((dist * 0.008) + 0.2, 1.0, 0.6);

        // NEW CODE FOR AUTO AIM

        // 1. Calculate Servo Position using the linear equation
        double targetAimPos = (0.0008702083 * x) + 0.3177145;

        // 2. Calculate Motor Power using the linear equation
        double targetPower = 0.4682029 + (0.001781158 * x) - (0.000005436021 * Math.pow(x, 2));

        // 3. SAFETY: Clip the values to make sure they stay within safe values
        double finalAim = Range.clip(targetAimPos, 0.3, 1.0);
        double finalPower = Range.clip(targetPower, 0.0, 1.0);

        // 4. Apply to hardware
        //aim.setPosition(finalAim);
        //shooter.setPower(finalPower);
        aim.setPosition(targetAimPos);
        shooter.setPower(targetPower);
    }
}