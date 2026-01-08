package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="BeeTeleop", group="Robot")
public class TeleOp extends OpMode {
private Follower follower;

public DcMotor shooterMotorL;
public DcMotor shooterMotorR;
public DcMotor intakeMotor;
public Servo intakeServo;
public Servo transferServo;





    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        //defines the motor(s) and servo(s)
        shooterMotorL = hardwareMap.get(DcMotor.class, "shooterMotorL");
        shooterMotorR = hardwareMap.get(DcMotor.class, "shooterMotorR");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        transferServo = hardwareMap.get(Servo.class, "transferServo");

        //setting directions
        shooterMotorR.setDirection(FORWARD);
        shooterMotorL.setDirection(REVERSE);
        intakeMotor.setDirection(REVERSE);
        intakeServo.setDirection(Servo.Direction.REVERSE);


        intakeServo.setPosition(0);


    }

    @Override
    public void start() {

        follower.startTeleOpDrive();

    }

    @Override
    public void loop() {
        follower.update();

        //joe builder
        follower.setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);

        //shooter motors will turn on/off
        //this speed is for shots closer to the goal
        if (gamepad1.dpad_down) {
            shooterMotorL.setPower(0.60);
            shooterMotorR.setPower(0.60);

            // automatic shooting
            sleep(2000);
            intakeServo.setPosition(0.6);
            sleep(500);
            intakeServo.setPosition(0);
            sleep(1000);
            intakeServo.setPosition(0.8);
            sleep(500);
            intakeServo.setPosition(0);
            sleep(500);

            // stop motors
            shooterMotorL.setPower(0);
            shooterMotorR.setPower(0);
        }
        //this speed is for shots further away from the goal
        if (gamepad1.dpad_up) {
            shooterMotorL.setPower(0.75);
            shooterMotorR.setPower(0.75);

            // automatic shooting
            sleep(2000);
            intakeServo.setPosition(0.6);
            sleep(500);
            intakeServo.setPosition(0);
            sleep(1000);
            intakeServo.setPosition(0.8);
            sleep(500);
            intakeServo.setPosition(0);
            sleep(500);

            // stop motors
            shooterMotorL.setPower(0);
            shooterMotorR.setPower(0);
        }
        if (gamepad1.xWasPressed()) {
            shooterMotorL.setPower(0.0);
            shooterMotorR.setPower(0.0);
        }

        //intake motor and servo will turn on/off at same time
        if (gamepad1.aWasPressed()) {
            intakeServo.setPosition(0);
            intakeMotor.setPower(1);
            transferServo.setPosition(0);
        }
        if (gamepad1.bWasPressed()) {
            intakeMotor.setPower(0);
            transferServo.setPosition(0.5);
        }
        if (gamepad1.dpad_left && intakeMotor.getPower() != 0) {
            intakeServo.setPosition(0);
        }
        if (gamepad1.dpad_right && intakeMotor.getPower() != 0) {
            intakeServo.setPosition(0.5);
        }
        if (gamepad1.yWasPressed()) {
            intakeMotor.setPower(-1);
        }
        if (gamepad1.left_bumper) {
            intakeServo.setPosition(1);
        }
    }

}
