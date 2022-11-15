package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive9974;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class TeleOpWithInlineArmMoveSolo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        waitForStart();

        MecanumDrive9974 drive = new MecanumDrive9974(hardwareMap);
       // drive.armMinPowerDuringHold = 0.176;
        PIDFCoefficients SHOOTER_PID = new PIDFCoefficients(30, 0, 0, 13);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.redLED1.setMode(DigitalChannel.Mode.OUTPUT);
        drive.redLED2.setMode(DigitalChannel.Mode.OUTPUT);
        drive.greenLED1.setMode(DigitalChannel.Mode.OUTPUT);
        drive.greenLED2.setMode(DigitalChannel.Mode.OUTPUT);

        int armStartPosition = drive.SlideMotor.getCurrentPosition();
        drive.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int startPosition=0;
        int poistionOfArm =0;
/*
        startPosition = drive.SlideMotor.getCurrentPosition();
        int endPosition = startPosition;
        int level1endPosition = startPosition + 76;
        int level2endPosition = startPosition + 104;
        int level3endPosition = startPosition + 176;
*/
       // drive.SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boolean gamepad2_Y_WasPressed = false;
        boolean gamepad2_A_WasPressed = false;
        boolean gamepad2_B_WasPressed = false;
        boolean gamepad2_X_WasPressed = false;

        while (opModeIsActive()) {
         if(gamepad1.right_trigger==0.0) {
             drive.SlideMotor.setPower(0.0);
             drive.rotorMotor.setPower(0.0);
             drive.setWeightedDrivePower(
                     new Pose2d(
                             -gamepad1.left_stick_y * .9,
                             -gamepad1.left_stick_x * .9,
                             -gamepad1.right_stick_x * .7
                     )
             );
         }
            //TODO create button map

            drive.update();
            /*
            if (gamepad2.right_trigger>0){
                drive.SlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                drive.SlideMotor.setPower(gamepad2.right_trigger*1.0);
                gamepad2_Y_WasPressed = false;
                gamepad2_A_WasPressed = false;
                gamepad2_B_WasPressed = false;
                gamepad2_X_WasPressed = false;
            }
            if (gamepad2.left_trigger>0){
                drive.SlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                drive.SlideMotor.setPower(gamepad2.left_trigger*1.0);
                gamepad2_Y_WasPressed = false;
                gamepad2_A_WasPressed = false;
                gamepad2_B_WasPressed = false;
                gamepad2_X_WasPressed = false;
            }
*/
            if (drive.digitalTouch.getState() == true) {
                telemetry.addData("Digital Touch", "Is Not Pressed");
                drive.redLED1.setState(true);
                drive.redLED2.setState(true);
                drive.greenLED1.setState(false);
                drive.greenLED2.setState(false);
            }
            else {
                drive.stopIntakeBlocks();
                drive.redLED1.setState(false);
                drive.redLED2.setState(false);
                drive.greenLED1.setState(true);
                drive.greenLED2.setState(true);

            }

            if (gamepad2.dpad_right) // X is intake system
                drive.spinwheelright();
            if (gamepad2.dpad_left) // X is intake system
                drive.spinwheelleft();
            if (gamepad1.dpad_up) {
                drive.stopIntakeBlocks();
            }
            if (gamepad2.dpad_down) // X is intake system
                drive.spinwheelstop();
            if(gamepad2.y)
            {
                gamepad2_Y_WasPressed = true;
                gamepad2_A_WasPressed = false;
                gamepad2_B_WasPressed= false;
                gamepad2_X_WasPressed = false;
            }

            if (gamepad2_Y_WasPressed) // X is intake system
            {

                if(drive.SlideMotor.getCurrentPosition() - drive.targetEncoderCountLevel3 < -100){
                    drive.SlideMotor.setPower(0.7);
                }
                else if(drive.SlideMotor.getCurrentPosition() - drive.targetEncoderCountLevel3 > 100){
                    drive.SlideMotor.setPower(-0.7);
                }
                else{
                    drive.SlideMotor.setPower(0.0);
                }
            }
            if(gamepad2.b)
            {
                gamepad2_Y_WasPressed = false;
                gamepad2_A_WasPressed = false;
                gamepad2_B_WasPressed= true;
                gamepad2_X_WasPressed = false;
            }
            if (gamepad2_B_WasPressed) // X is intake system
            {
              if(drive.SlideMotor.getCurrentPosition() - drive.targetEncoderCountLevel2 < -100){
                  drive.SlideMotor.setPower(0.7);
              }
             else if(drive.SlideMotor.getCurrentPosition() - drive.targetEncoderCountLevel2 > 100){
                  drive.SlideMotor.setPower(-0.7);
              }
             else{
                 drive.SlideMotor.setPower(0.0);
              }

                //drive.ArmLifter(2,4);
             //   double powerToApply = drive.ArmLifterPowerToApplyCalculation(2,4);
              //  drive.SlideMotor.setPower(powerToApply);
            }
            if(gamepad2.a)
            {
                gamepad2_Y_WasPressed = false;
                gamepad2_A_WasPressed = true;
                gamepad2_B_WasPressed= false;
                gamepad2_X_WasPressed = false;
            }
            if (gamepad2_A_WasPressed) // X is intake system
            {
                if(drive.SlideMotor.getCurrentPosition() - drive.targetEncoderCountLevel1 < -100){
                    drive.SlideMotor.setPower(0.7);
                }
                else if(drive.SlideMotor.getCurrentPosition() - drive.targetEncoderCountLevel1 > 100){
                    drive.SlideMotor.setPower(-0.7);
                }
                else{
                    drive.SlideMotor.setPower(0.0);
                }
            }
       /*
            if (gamepad1.x) // X is intake system
            {
                drive.ArmLifter(-1, 4);
            }

        */
            if (gamepad1.right_bumper) // X is intake system
            {
                drive.inTakeblocks();
            }

            if (gamepad1.left_bumper) // X is intake system
            {
                drive.outTakeblocks();

            }
            drive.update();
            if(gamepad2.x)
            {
                if(gamepad2_X_WasPressed)
                    gamepad2_X_WasPressed=false;
                else
                    gamepad2_X_WasPressed=true;

                if(gamepad2.left_stick_y > 0.0)
                    gamepad2_X_WasPressed=false;
            }
           if(gamepad1.right_trigger>0.0) {
               drive.setWeightedDrivePower(
                       new Pose2d(
                               -gamepad1.left_stick_y * 0,
                               -gamepad1.left_stick_x * 0,
                               -gamepad1.right_stick_x * 0
                       )
               );
               drive.rotorMotor.setPower(gamepad1.left_stick_x * -1.0);
               double powerToApply = 0.0;
               if (Math.abs(gamepad1.right_stick_y) > 0.00) {
                   gamepad2_Y_WasPressed = false;
                   gamepad2_A_WasPressed = false;
                   gamepad2_B_WasPressed = false;
                   gamepad2_X_WasPressed = false;
                   drive.SlideMotor.setPower(gamepad1.right_stick_y * -1.0);
               }
               if (!(gamepad2_A_WasPressed || gamepad2_B_WasPressed || gamepad2_X_WasPressed || gamepad2_Y_WasPressed)) {
                   drive.SlideMotor.setPower(gamepad1.right_stick_y * -1.0);
               }

           }

// Show the potentiometer’s voltage in telemetry
            telemetry.addData(("PostionLevel:"),poistionOfArm);
            telemetry.addData("Amr Power ", drive.SlideMotor.getPower());
            telemetry.addData("Position", drive.SlideMotor.getCurrentPosition());
            telemetry.update();
            telemetry.update();

        }
    }
}


