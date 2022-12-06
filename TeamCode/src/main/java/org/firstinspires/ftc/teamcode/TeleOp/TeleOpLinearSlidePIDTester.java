package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.AdrianControls.LinearSlidePID;
import org.firstinspires.ftc.teamcode.AdrianControls.LinearSlidePIDWithVelocity;
import org.firstinspires.ftc.teamcode.AdrianControls.MagneticSwitch;
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


public class TeleOpLinearSlidePIDTester extends LinearOpMode {
   // LinearSlidePID linearslide;
    LinearSlidePIDWithVelocity linearslide;
    @Override
    public void runOpMode() throws InterruptedException {
       // linearslide = newLinearSLidePID(hardwareMap);
        linearslide = new LinearSlidePIDWithVelocity(hardwareMap);
        waitForStart();
        //linearslide.setTargetPosition(0.25);
        linearslide.moveToLowPole();

        boolean hasItReachedTheTop = false;
        while(opModeIsActive()){
            if(!hasItReachedTheTop) {
                linearslide.update();
            }
            if((linearslide.getPosition() - 0.35) < 0.01)
            {
                hasItReachedTheTop = true;
            }
            if(hasItReachedTheTop)
            {
                linearslide.goToZero();

            }
            telemetry.addData("LiftPos", linearslide.getCurrentPosition());
            telemetry.update();

        }
       // drive.SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boolean gamepad2_Y_WasPressed = false;
        boolean gamepad2_A_WasPressed = false;
        boolean gamepad2_B_WasPressed = false;
        boolean gamepad2_X_WasPressed = false;


    }
}


