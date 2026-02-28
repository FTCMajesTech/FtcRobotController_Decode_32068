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
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name="ShooterTest", group="Robot")
public class ShooterTest extends OpMode {
    // --- HARDWARE ---
    private Follower follower;
    private Limelight3A limelight;
    private IMU imu;
    public DcMotorEx shooter;
    public DcMotor intake, transfer, backTransfer;
    public Servo aim, gate;

    // --- TUNING CONSTANTS ---
    // SHOOTER MOTOR (5800 RPM Bare Motor)
    final double MAX_TICKS_PER_SECOND = 2500; // Calculation: 5800RPM / 60 * 28 ticks/rev = ~2706 ticks/sec
                                              // We set MAX slightly lower to give room for PID correction

    // AUTO-AIM PID
    final double kP_AIM = 0.025; // kP (Proportional) gain: Higher = faster turn, but might oscillate
    final double MIN_AIM_POWER = 0.05; // Minimum power to overcome friction

    // --- VARIABLES ---
    private boolean centric = false;
    private boolean yPressedLastIteration = false;

    // --- Telemetry Trackers ---
    private double currentTargetAim = 0;
    double targetVelocity = 0;



    @Override
    public void init() {
        // --- Follower & Drive ---
        follower = Constants.createFollower(hardwareMap);
        follower.update();
        follower.setStartingPose(Constants.FCDPose);


        // --- Shooter Initialization (CRITICAL SECTION) ---
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
        shooter.setVelocityPIDFCoefficients(160.0, 0.2, 18.0, 14.2); // P=10/11, I=0.1, D=0, F=13

        // --- Servos ---
        aim = hardwareMap.get(Servo.class, "aim");
        aim.setPosition(1);
        gate = hardwareMap.get(Servo.class, "gate");
        gate.setPosition(0.25); //0.32 is old close, new gate close is 0.25 , new gate open is 0.43, old gate open is 0.1

        // --- Intake/Transfer ---
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(REVERSE);
        intake.setPower(0);
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(FORWARD);
        transfer.setPower(0);
        backTransfer = hardwareMap.get(DcMotorEx.class, "backTransfer");
        backTransfer.setDirection(REVERSE);
        backTransfer.setPower(0);

        // --- Sensors ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // pipeline 0= AprilTag detection, pipeline 1= color detection, pipeline 2= backup/testing
        limelight.start();

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot) );

        telemetry.setMsTransmissionInterval(11);
    }



    @Override
    public void start() {
        follower.startTeleOpDrive();
    }



    @Override
    public void loop() {
        //Call this once per loop
        follower.update();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());


        // --- INPUTS ---
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

        // Switching between field centric and robot centric
        boolean yIsPressed = gamepad1.y;
        if (yIsPressed && !yPressedLastIteration) {
            centric = !centric; // Need to fix issue of initialize messing up field centric
        }
        yPressedLastIteration = yIsPressed;

        // --- SENSOR DATA ---
        LLResult result = limelight.getLatestResult();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        // --- MANUAL OVERRIDES
        if (gamepad1.bWasPressed()) { // Manual shooter shutdown
            shooter.setVelocity(0);
            aim.setPosition(1);
            gate.setPosition(0.25);
        }

        if (gamepad1.aWasPressed()) { // Intake on
            intake.setPower(.75);
            transfer.setPower(1);
        }

        if (gamepad1.xWasPressed()) { // Intake off
            intake.setPower(0);
            transfer.setPower(0);
            backTransfer.setPower(0);
        }

        if (gamepad1.dpad_left) { // Manaul override for gate to open
            gate.setPosition(0.43);
        }

        // THIS IS WHERE TO CHANGE VALUES TO MAKE EQUATIONS FOR SHOOTER!!!!

        if (gamepad1.yWasPressed()) {
            gate.setPosition(0.43);
            shooter.setVelocity(1450);
            aim.setPosition(0.45);
            intake.setPower(0.75);
            transfer.setPower(1);
            backTransfer.setPower(1);
        }



        // --- APPLY DRIVE ---
        // Single point of truth for movement
        follower.setTeleOpDrive(drive, strafe, rotate, centric);



        // --- TELEMETRY ---
        if (result != null && result.isValid()) {
            double distanceFromGoal = getDistanceFromTage(result.getTa());

            telemetry.addData("Mode", "AUTO-AIM ACTIVE");
            telemetry.addData("Distance", distanceFromGoal);
            telemetry.addData("Target X", result.getTx());
            telemetry.addData("Calc Aim Pos", "%.3f", currentTargetAim);
            telemetry.addData("Target Velocity", "%.3f", targetVelocity);
            telemetry.addData("Actual Velocity", "%.3f", shooter.getVelocity());
        } else {
            telemetry.addData("Mode", "Manual Drive");
            telemetry.addData("Field Centric", !centric);
        }
        telemetry.update();
    }



    // --- HELPER METHODS ---
    public double getDistanceFromTage(double ta) {
        double scale = 18398.87;
        return Math.pow(scale / ta,(1/1.8925));
    }
}