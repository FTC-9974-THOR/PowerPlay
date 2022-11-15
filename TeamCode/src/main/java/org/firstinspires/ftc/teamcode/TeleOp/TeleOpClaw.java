package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AdrianControls.Claw;
import org.firstinspires.ftc.teamcode.AdrianControls.LinearSlidePIDWithVelocity;
import org.firstinspires.ftc.teamcode.AdrianControls.Turret;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Disabled


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")


public class TeleOpClaw extends LinearOpMode {
   // LinearSlidePID linearslide;
    LinearSlidePIDWithVelocity linearslide;
    Claw claw;
    Turret turret;
    @Override
    public void runOpMode() throws InterruptedException {
        claw = new Claw(hardwareMap);
        turret = new Turret(hardwareMap);
       // linearslide = newLinearSLidePID(hardwareMap);
        linearslide = new LinearSlidePIDWithVelocity(hardwareMap);
        waitForStart();
        //linearslide.setTargetPosition(0.25);
        //linearslide.moveTo(0.35);

        boolean isClawOpened = true;
        claw.moveClawOpen(true);
        while(opModeIsActive()){
            //linearslide.update();
            if(isClawOpened)
            {
                isClawOpened = false;
                claw.moveClawOpen(false);
                linearslide.moveTo(0.35);
                turret.turretGoHome(1);
            }
            telemetry.addData("LiftPos", linearslide.getCurrentPosition());
            telemetry.update();
            turret.update();
            linearslide.update();

        }
       // drive.SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boolean gamepad2_Y_WasPressed = false;
        boolean gamepad2_A_WasPressed = false;
        boolean gamepad2_B_WasPressed = false;
        boolean gamepad2_X_WasPressed = false;


    }
}


