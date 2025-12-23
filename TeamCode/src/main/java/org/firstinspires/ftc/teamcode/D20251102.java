package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="teleop", group="Robot")
@Disabled
public class D20251102 extends LinearOpMode {
    // VARIABLES:
    // DriveTrain
    public DcMotor  FrontLeftDrive;
    public DcMotor  FrontRightDrive;
    public DcMotor  BackLeftDrive;
    public DcMotor BackRightDrive;
    public DcMotor ShooterMotor1;
    public DcMotor ShooterMotor2;

    double DriveSpeed = 0.5;

    // Odometry
    GoBildaPinpointDriver stalker;

    @Override
    public void runOpMode() {
        // DEFINE ELECTRONICS:
        // DriveTrain
        FrontLeftDrive  = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "FrontRightDrive");
        BackLeftDrive = hardwareMap.get(DcMotor.class,"BackLeftDrive");
        BackRightDrive = hardwareMap.get(DcMotor.class, "BackRightDrive");

        // Odometry
        stalker = hardwareMap.get(GoBildaPinpointDriver.class, "stalker");

        // INITIAL SETUPS:
        // DriveTrain
        FrontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        FrontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        BackLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        FrontLeftDrive.setZeroPowerBehavior(BRAKE);
        FrontRightDrive.setZeroPowerBehavior(BRAKE);
        BackLeftDrive.setZeroPowerBehavior(BRAKE);
        BackRightDrive.setZeroPowerBehavior(BRAKE);

        // Odometry Stuff:
        stalker.setOffsets(-169.0, -208.0, DistanceUnit.MM);
        stalker.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        stalker.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Odometry Starting POS
        stalker.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, -923.925, 1601.47, AngleUnit.RADIANS, 0);

        // Odometry Telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X Offset", stalker.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y Offset", stalker.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Scalar", stalker.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            FrontLeftDrive.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * DriveSpeed);
            FrontRightDrive.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * DriveSpeed);
            BackLeftDrive.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * DriveSpeed);
            BackRightDrive.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * DriveSpeed);

            // moveRobot();

            Pose2D pos = stalker.getPosition();
            telemetry.addData("Robot X", stalker.getPosX(DistanceUnit.MM));
            telemetry.addData("Robot Y", stalker.getPosY(DistanceUnit.MM));
            telemetry.addData("Robot Heading", pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("raw X", stalker.getEncoderX());
            telemetry.addData("raw Y", stalker.getEncoderY());
            telemetry.update();
            stalker.update();

            if(gamepad1.x) {
               stalker.resetPosAndIMU();
            }

            System.out.print("Raw X: " + stalker.getEncoderX());
            System.out.println("Raw Y: " + stalker.getEncoderY());
        }
    }

}
