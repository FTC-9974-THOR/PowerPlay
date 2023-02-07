package org.firstinspires.ftc.teamcode.AdrianControls;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.util.MathUtilities;

public class Claw {
    public double CLAW_OPEN = MathUtilities.map(1100, 500, 2500, 0,1),
            CLAW_CLOSED = MathUtilities.map(1465, 500, 2500, 0,1),
    //Claw 1 Positions
    CLAW_OPEN_1 = MathUtilities.map(1600, 500, 2500, 0,1),
    CLAW_CLOSED_1 = MathUtilities.map(1920, 500, 2500, 0,1), //1970 orig 17th of Jan
    //Claw 2 Positions
    CLAW_OPEN_2 = MathUtilities.map(1500, 500, 2500, 0,1),
    CLAW_CLOSED_2 = MathUtilities.map(1865, 500, 2500, 0,1);
    public enum statesWithinClaw
    {
        linearSlideDown,
        clawOpen,
        linearSlideReturn,
        clawOpenDone,
        IDLE
    }
    public statesWithinClaw clawMode = statesWithinClaw.IDLE;
    @Hardware(name = "ClawServo")

    public Servo clawServo;
    private LinearSlidePIDWithVelocity linearSlide;
    //Change this when you change Claw to pick which Claw is on.
    public int clawNum = 1;
// We are getting the state of the magneticLimitSwitch(True means it's not touched)
    public Claw(HardwareMap hardwareMap, LinearSlidePIDWithVelocity linearSlide) {
        Realizer.realize(this, hardwareMap);
        this.linearSlide = linearSlide;
        this.clawNum = clawNum;
        if(clawNum == 1)
        {
            CLAW_OPEN = CLAW_OPEN_1;
            CLAW_CLOSED = CLAW_CLOSED_1;
        }
        else
        {
            CLAW_OPEN = CLAW_OPEN_2;
            CLAW_CLOSED = CLAW_CLOSED_2;

        }
    }
// This is a function that is called in autonomous, but could be called in Teleop too
//Basically, I am taking the position you want to go to, either one or two, and using that to determine the power of the motor.
    /*
    public void moveClawOpen( boolean open) {
        if (open == true)
        {
            clawServo.setPosition(CLAW_OPEN);

        }
        else
        {
            clawServo.setPosition(CLAW_CLOSED);
        }
    }
     */
    public void OpenClaw()
    {
        clawServo.setPosition(CLAW_OPEN);
        clawMode = statesWithinClaw.IDLE;
    }
    public void OpenClawWithLinearSlide(double subtractForLower)
    {
        if(subtractForLower == linearSlide.subtractionForLowerCalcualtedHeightFast)
        {
            OpenClaw();
            clawMode = statesWithinClaw.clawOpenDone;
            return;
        }
        switch(clawMode)
        {
            case IDLE:
                break;
            case linearSlideDown:
                linearSlide.moveToBelowOriginalLevel(subtractForLower);
                clawMode = statesWithinClaw.clawOpen;
                break;
            case clawOpen:
                if(!linearSlide.isBusy())
                {
                    clawServo.setPosition(CLAW_OPEN);
                    linearSlide.moveToOriginalLevel();
                    clawMode = statesWithinClaw.linearSlideReturn;
                }
                break;
            case linearSlideReturn:
                if(!linearSlide.isBusy())
                {
                    clawMode = statesWithinClaw.clawOpenDone;
                    linearSlide.resetOrignalLevelValues();
                }
                break;
            case clawOpenDone:
                break;
        }
    }

    public void CloseClaw()
    {
        clawServo.setPosition(CLAW_CLOSED);
    }

}

