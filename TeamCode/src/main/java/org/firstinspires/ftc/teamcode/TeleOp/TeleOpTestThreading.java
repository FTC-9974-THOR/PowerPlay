package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive9974;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
@Disabled
public class TeleOpTestThreading extends OpMode {

    @Override
    public void init() {
        // drive.armMinPowerDuringHold = 0.176;
        MecanumDrive9974 drive = new MecanumDrive9974(hardwareMap);

        PIDFCoefficients SHOOTER_PID = new PIDFCoefficients(30, 0, 0, 13);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int armStartPosition = drive.SlideMotor.getCurrentPosition();
        drive.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int startPosition = 0;
        int poistionOfArm = 0;
    }
    @Override
    public void init_loop() {

    }
    @Override
    public void start() {

    }
    @Override
    public void loop() {
        MecanumDrive9974 drive = new MecanumDrive9974(hardwareMap);

        drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y*.9,
                            -gamepad1.left_stick_x*.9,
                            -gamepad1.right_stick_x*.7
                    )
            );
            //TODO create button map

            drive.update();
            if (gamepad1.right_trigger>0){
                drive.SlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                drive.SlideMotor.setPower(gamepad1.right_trigger*0.35);
            }
            if (gamepad1.left_trigger>0){
                drive.SlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                drive.SlideMotor.setPower(gamepad1.left_trigger*0.3);
            }

            if (drive.digitalTouch.getState() == true) {
                telemetry.addData("Digital Touch", "Is Not Pressed");
            }
            else {
                drive.stopIntakeBlocks();
            }

            if (gamepad2.dpad_right) // X is intake system
                drive.spinwheelright();
            if (gamepad2.dpad_left) // X is intake system
                drive.spinwheelleft();
            if (gamepad2.dpad_up) {
                drive.stopIntakeBlocks();
            }
            if (gamepad2.dpad_down) // X is intake system
                drive.spinwheelstop();
            if (gamepad2.y) // X is intake system
            {
             drive.ArmLifter(3,4);

                drive.RotorArmStop();
                drive.outTakeblocks();

                drive.stopIntakeBlocks();
                drive.RotorArmFunctionBack();

                drive.RotorArmStop();
                drive.SlideMotor.setPower(0.0);
            }
            if (gamepad2.b) // X is intake system
            {
                drive.ArmLifter(2,4);
                drive.RotorArmFunctionGo();

                drive.RotorArmStop();
                drive.outTakeblocks();

                drive.stopIntakeBlocks();
                drive.RotorArmFunctionBack();

                drive.RotorArmStop();
                drive.SlideMotor.setPower(0.0);
            }
            if (gamepad2.a) // X is intake system
            {
                drive.ArmLifter(1,4);
                drive.RotorArmFunctionGo();

                drive.RotorArmStop();
                drive.outTakeblocks();

                drive.stopIntakeBlocks();
                drive.RotorArmFunctionBack();

                drive.RotorArmStop();
                drive.SlideMotor.setPower(0.0);
            }
       /*
            if (gamepad1.x) // X is intake system
            {
                drive.ArmLifter(-1, 4);
            }

        */
            if (gamepad2.right_bumper) // X is intake system
            {
                drive.inTakeblocks();
            }

            if (gamepad2.left_bumper) // X is intake system
            {
                drive.outTakeblocks();

            }
            drive.update();
            if (gamepad2.b) {// B is stopping intake

            }
            /*
            shooterServo positions
            0 = LOAD
            1 = FIRE
             */




                    drive.update();


            //drive.Arm(gamepad2.right_stick_y/2); // Right stick down pulls arm up, and vice versa


            /*
            Arm Controller
             */
          //  drive.arm.setPower(-gamepad2.left_stick_y*.4);


       //     if (gamepad2.dpad_down) {


        //    }




// Show the potentiometer’s voltage in telemetry
            telemetry.addData("Position", drive.SlideMotor.getCurrentPosition());
            telemetry.update();
            telemetry.update();

        }
    @Override
    public void stop() {
    }
    }



