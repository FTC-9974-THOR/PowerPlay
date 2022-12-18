package org.firstinspires.ftc.teamcode.AdrianControls;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.util.MathUtilities;


public class RotatingArm {
    public static final double inputMin = 500;
    public static final double inputMax = 2500;
    public static final double outputMin = 0;
    public static final double outputMax = 1;
    public double minimumRawValueForServoMotion = 0;
    static final double minimumRawValueForServoMotionAuto = MathUtilities.map(860, inputMin, inputMax, outputMin,outputMax);
    public static final double minimumRawValueForServoMotionTeleop = MathUtilities.map(506, inputMin, inputMax, outputMin,outputMax);

    private LinearSlidePIDWithVelocity linearSlide;
    public enum RotatorArmStates
    {
        Idle,
        Moving,
        Teleop
    }


    public RotatorArmStates RotatorArmMode;
    public static final double bottomTickValue = 1555;
    public static final double nintyDegreeTickDisplacementRight = 376;
    public static final double  nintyDegreeTickDisplacementLeft = 375; //370.53;
    public static final double
            ROTATOR_LEFT = MathUtilities.map(bottomTickValue-nintyDegreeTickDisplacementLeft, inputMin, inputMax, outputMin,outputMax),
            ROTATOR_RIGHT = MathUtilities.map(bottomTickValue+nintyDegreeTickDisplacementRight, inputMin, inputMax, outputMin,outputMax),
            ROTATOR_BOTTOM= MathUtilities.map(bottomTickValue, inputMin, inputMax, outputMin,outputMax),
            ROTATOR_TOP = MathUtilities.map(2482, inputMin, inputMax, outputMin,outputMax);


    private double positionTarget = ROTATOR_BOTTOM;
    private double currentPosition = ROTATOR_BOTTOM;
    @Hardware(name = "RotatingArmServo")

    public Servo rotatingArmServo;

    public RotatingArm(HardwareMap hardwareMap,LinearSlidePIDWithVelocity linearSlide) {
        Realizer.realize(this, hardwareMap);
        this.linearSlide = linearSlide;
    }

    public double getRotatorArmPositionTick()
    {
        return MathUtilities.map(rotatingArmServo.getPosition(),outputMin,outputMax,inputMin,inputMax);
    }
    public double getRotatorArmPositionRaw()
    {
        return rotatingArmServo.getPosition();
    }
    public void setRotatorArmPositionRaw(double position)
    {
        RotatorArmMode = RotatorArmStates.Moving;
        positionTarget = position;

    }
    public void setRotatorArmPositionTick(double position)
    {
        RotatorArmMode = RotatorArmStates.Moving;
        positionTarget = postionCalculator(position);

    }
    public void rotateArmToPosition( double positionBasedOnTester) {
            rotatingArmServo.setPosition(positionBasedOnTester);
    }
    public double postionCalculator(double inputCount){
        return MathUtilities.map(inputCount,inputMin,inputMax,outputMin,outputMax);
    }
    public boolean isBusy()
    {
        return RotatorArmMode != RotatorArmStates.Idle;
    }

    public void update(){
        if(RotatorArmMode == RotatorArmStates.Teleop)
        {
            minimumRawValueForServoMotion = minimumRawValueForServoMotionTeleop;
        }
        else
        {
            minimumRawValueForServoMotion = minimumRawValueForServoMotionAuto;
        }
        if(Math.abs(positionTarget - currentPosition)<=minimumRawValueForServoMotion)
        {
            rotatingArmServo.setPosition(positionTarget);
            RotatorArmMode = RotatorArmStates.Idle;
        }
        else {
            if (linearSlide.isLiftAboveCameraHeight()) {
                if(positionTarget > currentPosition)
                {
                    currentPosition += minimumRawValueForServoMotion;
                    rotatingArmServo.setPosition(currentPosition);
                }
                else
                {
                    currentPosition -= minimumRawValueForServoMotion;
                    rotatingArmServo.setPosition(currentPosition);
                }
                //rotatingArmServo.setPosition(positionTarget);
            }
        }
    }

}

