package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp3", group="Robot")
public class TeleOp3 extends OpMode {
private Follower follower;
private Limelight3A limelight;
public DcMotor shooter;
private double power = 0.25;




    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        shooter = hardwareMap.get(DcMotor.class, "shooter");

        shooter.setDirection(FORWARD);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

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

        //VANILLER CHANGE THE COMMENTS TO BE GOOD
        LLResult result = limelight.getLatestResult();

        if (gamepad1.dpad_up) {
            shooter.setPower(0.65);
        }
        if (gamepad1.dpad_down) {
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
                telemetry.addData("april tag", result.getFiducialResults());
            }
        }
    }

    private void spinRight() {
        // Arguments: (forward, strafe, rotation)
        // 0 forward, 0 strafe, and positive power for rotation
        follower.setTeleOpDrive(0, 0, -power);
    }

    private void spinLeft() {
        // 0 forward, 0 strafe, and negative power for rotation
        follower.setTeleOpDrive(0, 0, power);
    }

    private void brake() {
        // Sets all movement vectors to zero
        follower.setTeleOpDrive(0, 0, 0);
    }

}
