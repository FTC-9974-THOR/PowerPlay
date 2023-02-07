package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.AdrianControls.Claw;
import org.firstinspires.ftc.teamcode.AdrianControls.LinearSlidePIDWithVelocity;
import org.firstinspires.ftc.teamcode.AdrianControls.RotatingArm;
import org.firstinspires.ftc.teamcode.AdrianControls.Turret;
import org.firstinspires.ftc.teamcode.AdrianControls.TurretWithPid;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive9974;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "Drive_1")
public class TeleOpPowerPlay9974 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LinearSlidePIDWithVelocity linearslide;
        Claw claw;
        Turret turret;
        //TurretWithPid turretWithPID = new TurretWithPid(hardwareMap);
        RotatingArm rotatorArm;
        waitForStart();
        linearslide = new LinearSlidePIDWithVelocity(hardwareMap);
        claw = new Claw(hardwareMap,linearslide);
        MecanumDrive9974 drive = new MecanumDrive9974(hardwareMap);
        turret = new Turret(hardwareMap);
        rotatorArm = new RotatingArm(hardwareMap,linearslide);
        // drive.armMinPowerDuringHold = 0.176;
        PIDFCoefficients SHOOTER_PID = new PIDFCoefficients(30, 0, 0, 13);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       // drive.SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


      //  int armStartPosition = drive.SlideMotor.getCurrentPosition();
        int startPosition=0;
        int poistionOfArm =0;
        double maxSpeedDriveTrainPercentage = 0.50;
        double maxSpeedDriveTrainPercentageForTurn = 0.40;

       // drive.SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boolean gamepad2_Y_WasPressed = false;
        boolean gamepad2_A_WasPressed = false;
        boolean gamepad2_B_WasPressed = false;
        boolean gamepad2_X_WasPressed = false;

        rotatorArm.rotatingArmServo.setPosition(rotatorArm.ROTATOR_BOTTOM);
        //turretWithPID.turretGoHome();
        while (opModeIsActive()) {
            if(gamepad2.right_stick_y!=0)
            {

                linearslide.setTargetPosition(linearslide.getCurrentPosition()+(gamepad2.right_stick_y*-0.05));
            }
            linearslide.update();
            if(gamepad1.right_trigger>0.0) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * (maxSpeedDriveTrainPercentage + gamepad1.right_trigger * (1-maxSpeedDriveTrainPercentage)),
                                -gamepad1.left_stick_x * (maxSpeedDriveTrainPercentage + gamepad1.right_trigger * (1-maxSpeedDriveTrainPercentage)),
                                -gamepad1.right_stick_x * maxSpeedDriveTrainPercentageForTurn
                        )
                );
            }
            else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * maxSpeedDriveTrainPercentage,
                                -gamepad1.left_stick_x * maxSpeedDriveTrainPercentage,
                                -gamepad1.right_stick_x * maxSpeedDriveTrainPercentageForTurn
                        )
                );
            }
            if(gamepad1.left_bumper) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * maxSpeedDriveTrainPercentage,
                                0,
                                -gamepad1.right_stick_x * maxSpeedDriveTrainPercentage
                        )
                );

            }
            if(gamepad1.right_bumper) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                 0,
                                -gamepad1.left_stick_x * .7,
                                -gamepad1.right_stick_x * .7
                        )
                );

            }
            //TODO create button map

            drive.update();

            if(gamepad2.right_bumper)
            {
             claw.OpenClaw();
            }
            if(gamepad2.left_bumper)
            {
                claw.CloseClaw();
            }
            if(gamepad2.a)
            {
                if(gamepad2.right_trigger>0) {
                    linearslide.moveToLowPole();
                }
                else
                {
                    linearslide.moveToLowPoleTeleop();
                }
            }

            if(gamepad2.b)
            {
                if(gamepad2.right_trigger>0) {
                    linearslide.moveToMiddlePole();
                }
                else
                {
                    linearslide.moveToMiddlePoleTeleop();
                }
            }

            if(gamepad2.y)
            {
                if(gamepad2.right_trigger>0) {
                    linearslide.moveToHighPole();
                }
                else
                {
                    linearslide.moveToHighPoleTeleop();
                }
            }

            if(gamepad2.x)
            {

                linearslide.goToZero();
            }
            if(gamepad2.dpad_down)
            {

                linearslide.moveToAboveTheCameraHeight();
                rotatorArm.RotatorArmMode = RotatingArm.RotatorArmStates.Moving;
                rotatorArm.setRotatorArmPositionRaw(rotatorArm.ROTATOR_BOTTOM);
                //turret.turretGoHomeWithVoltage();
            }
          if(gamepad2.right_trigger>0) {
              if (gamepad2.dpad_right) {
                  rotatorArm.RotatorArmMode = RotatingArm.RotatorArmStates.Moving;
                  rotatorArm.setRotatorArmPositionRaw(rotatorArm.ROTATOR_RIGHT);

              }
              if (gamepad2.dpad_left) {
                  rotatorArm.RotatorArmMode = RotatingArm.RotatorArmStates.Moving;
                  rotatorArm.setRotatorArmPositionRaw(rotatorArm.ROTATOR_LEFT);

              }
              if (gamepad2.dpad_up) {
                  rotatorArm.RotatorArmMode = RotatingArm.RotatorArmStates.Moving;
                  rotatorArm.setRotatorArmPositionRaw(rotatorArm.ROTATOR_BOTTOM);

              }
          }

            if(!(gamepad2.left_trigger>0)) {
                if(Math.abs(gamepad2.left_stick_x)>0.0){
                    rotatorArm.RotatorArmMode = RotatingArm.RotatorArmStates.Teleop;
                    rotatorArm.setRotatorArmPositionTick(rotatorArm.getRotatorArmPositionTick() + gamepad2.left_stick_x * 10);
                }
            }
            rotatorArm.update();
            /*
            if(linearslide.getPosition()>0.1 && gamepad2.left_trigger>0) {
                if (gamepad2.left_stick_x > 0) {
                    turret.turretMotor.setPower(0.2);
                    turret.TurretMode = Turret.TurretStates.Teleop;

                } else if (gamepad2.left_stick_x < 0) {
                    turret.turretMotor.setPower(-0.2);
                    turret.TurretMode = Turret.TurretStates.Teleop;

                } else {
                    turret.turretMotor.setPower(0.0);
                    turret.TurretMode = Turret.TurretStates.Idle;

                }
            }
            else
            {
                turret.turretMotor.setPower(0.0);
            }
            */
            /*
            if(gamepad2.x)
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

                //drive.ArmLifter(2, 4);
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
       */
       /*
            if (gamepad1.x) // X is intake system
            {
                drive.ArmLifter(-1, 4);
            }


            if(gamepad2.x)
            {
                if(gamepad2_X_WasPressed)
                    gamepad2_X_WasPressed=false;
                else
                    gamepad2_X_WasPressed=true;

                if(gamepad2.left_stick_y > 0.0)
                    gamepad2_X_WasPressed=false;
            }

         */

            double powerToApply = 0.0;
            //turret.update();
            //turretWithPID.update();
            if(Math.abs(gamepad2.right_stick_y) > 0.00){
                gamepad2_Y_WasPressed = false;
                gamepad2_A_WasPressed = false;
                gamepad2_B_WasPressed= false;
                gamepad2_X_WasPressed = false;
                //drive.SlideMotor.setPower(gamepad2.right_stick_y*0.8);
            }
            if(!(gamepad2_A_WasPressed || gamepad2_B_WasPressed || gamepad2_X_WasPressed ||gamepad2_Y_WasPressed))
            {
                //drive.SlideMotor.setPower(gamepad2.right_stick_y*0.8);
            }



// Show the potentiometer’s voltage in telemetry

            telemetry.addData("JoystickValue", gamepad2.right_stick_y);
            telemetry.addData("LiftValue", linearslide.getCurrentPosition());
            telemetry.addData("TUrrentXValue",gamepad2.left_stick_x);
            telemetry.addData("TurrentMAGSWITCHValue",turret.magneticLimitSwitch.getState());
            telemetry.addData("TurretPotentiometerVoltage", turret.getTurretPotentiometerVoltage());
            telemetry.addData("isMotionProfillingBeingUsed", linearslide.isMotionProfillingBeingUsed);
            telemetry.addData("PositionInTicks", rotatorArm.getRotatorArmPositionTick());
            telemetry.addData("XValueForPID",-gamepad1.left_stick_y * maxSpeedDriveTrainPercentage);
            telemetry.addData("YValueForPID",-gamepad1.left_stick_x * maxSpeedDriveTrainPercentage);
            telemetry.addData("MaxHeading",-gamepad1.right_stick_x * maxSpeedDriveTrainPercentageForTurn);


            telemetry.update();
            telemetry.update();


        }
    }
}


