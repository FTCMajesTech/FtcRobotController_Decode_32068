package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp3", group="Robot")
public class TeleOp3 extends OpMode {
private Follower follower;
private Limelight3A limelight;
public DcMotor shooter;
public DcMotor intake;
public Servo servo1;
public Servo servo2;
private double power = 0.25;
//how fast robot rotates when auto-aligning



    @Override
    public void init() {
        //creates drive follower
        follower = Constants.createFollower(hardwareMap);
        follower.update();

        //connects shooter and sets direction
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter.setDirection(FORWARD);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(REVERSE);

        //connects limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

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
        servo1.setPosition(0);
        servo2.setPosition(0);

    }

    @Override
    public void loop() {
        follower.update();

        //updates drive system(left stick Y: forward/back, left stick X: strafe, right stick X: turn)
        follower.setTeleOpDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);

        //gets latest vision result
        LLResult result = limelight.getLatestResult();

        if (gamepad1.dpad_up) {
            shooter.setPower(0.65);
        }
        if (gamepad1.dpad_down) {
            shooter.setPower(0);
        }

        if (gamepad1.y) {
            intake.setPower(0.75);
        }
        if (gamepad1.x) {
            intake.setPower(0);
        }

        if (gamepad1.dpad_up) {
            servo1.setPosition(1);
        }
        if (gamepad1.dpad_down) {
            servo1.setPosition(0);
        }

        if (gamepad1.dpad_left) {
            servo2.setPosition(1);
        }
        if (gamepad1.dpad_right) {
            servo2.setPosition(0);
        }


        //tries to rotate until Tx is 0 and uses a while loop
        //can also freeze robot (CHANGE THIS CODE)
        //while loops can lock robot
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

        //Tx:how far the target is left/right
        //if target is left: spin left
        //if target is more right: spin right
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

    //helper movement methods
    private void spinRight() {
        // Arguments: (forward, strafe, rotation)
        // 0 forward, 0 strafe, and positive power for rotation
        follower.setTeleOpDrive(0, 0, -power);
    }

    private void spinLeft() {
        // 0 forward, 0 strafe, and negative power for rotation
        follower.setTeleOpDrive(0, 0, power);
    }

    //stop all movement
    private void brake() {
        // Sets all movement vectors to zero
        follower.setTeleOpDrive(0, 0, 0);
    }

}
