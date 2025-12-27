package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp2", group="Robot")
public class TeleOp2 extends OpMode {
private Follower follower;
private Limelight3A limelight;
public DcMotor shooter;
private double power = 0.25;

DcMotor FL;
DcMotor FR;
DcMotor BL;
DcMotor BR;



    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        shooter = hardwareMap.get(DcMotor.class, "shooter");

        shooter.setDirection(FORWARD);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        FL = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
        FR = hardwareMap.get(DcMotor.class, "FrontRightDrive");
        BL = hardwareMap.get(DcMotor.class, "BackLeftDrive");
        BR = hardwareMap.get(DcMotor.class, "BackRightDrive");
    }

    @Override
    public void start() {

        follower.startTeleOpDrive();
        limelight.start();

    }

    @Override
    public void loop() {
        follower.update();

        //joe builder
        follower.setTeleOpDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);

        LLResult result = limelight.getLatestResult();

        if (gamepad1.dpad_up) {
            shooter.setPower(0.60);
        }
        if (gamepad1.xWasPressed()) {
            shooter.setPower(0);
        }

        if (gamepad1.b) {
            while (result.getTx() != 0) {
                if (result.getTx() < 0) {
                    spinLeft();
                } else if (result.getTx() > 0) {
                    spinRight();
                } else {
                    brake();
                }
            }
        }

        if (gamepad1.a) {
            if (result.getTx() < 0) {
                spinLeft();
            } else if (result.getTx() > 0) {
                spinRight();
            } else {
                brake();
            }
        }


        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());
            }
        }
    }

    private void spinRight() {
        BL.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.REVERSE);
        FL.setPower(power); BL.setPower(power); FR.setPower(-power); BR.setPower(-power);
    }

    private void spinLeft() {
        BL.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.REVERSE);
        FL.setPower(-power); BL.setPower(-power); FR.setPower(power); BR.setPower(power);
    }

    private void brake() {
        BL.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.REVERSE);
        FL.setPower(0); BL.setPower(0); FR.setPower(0); BR.setPower(0);
    }

}
