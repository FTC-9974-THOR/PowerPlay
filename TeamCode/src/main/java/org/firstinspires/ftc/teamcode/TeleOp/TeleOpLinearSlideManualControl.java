package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AdrianControls.LinearSlidePID;
import org.firstinspires.ftc.teamcode.AdrianControls.LinearSlidePIDWithVelocity;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
//@Disabled
public class TeleOpLinearSlideManualControl extends LinearOpMode {
    LinearSlidePID linearslide;
    //LinearSlidePIDWithVelocity linearslide;
    @Override
    public void runOpMode() throws InterruptedException {
        linearslide = new LinearSlidePID(hardwareMap);
        //linearslide = new LinearSlidePIDWithVelocity(hardwareMap);
        waitForStart();
        //linearslide.setTargetPosition(0.25);
        //linearslide.moveTo(0.35);

        boolean hasItReachedTheTop = false;
        while(opModeIsActive()){
            linearslide.update();
            if(gamepad2.right_stick_y!=0)
            {
                linearslide.setTargetPosition(linearslide.getCurrentPosition()+(gamepad2.right_stick_y*-0.05));
            }
            if(gamepad2.b)
            {
                //linearslide.moveToMiddlePole();
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


