package org.firstinspires.ftc.teamcode.AdrianControls;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionSegment;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ftc9974.thorcore.control.VoltageRegulator;
import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.util.MotorUtilities;

import java.util.Collections;

import static java.util.Collections.singletonList;

public class LinearSlidePIDWithVelocity {

    @Hardware(name = "SlideMotor")
    public DcMotorEx Lift;
    @Hardware(name = "LiftMagneticSwitch")
    public DigitalChannel liftMagneticSwitch;

    private static final double diameterOfSpool = 0.03225; // in meters- diameter.
    private static final double metersPerTick = (Math.PI * diameterOfSpool) / (7 * 4 * 19.2);
    private static final double minHeight = 0; // meters
    private static final double maxHeight = 0.79; // also meters
    private static final double MAX_VEL = 3.5;
    private static final double MAX_ACCEL = 2.0;//0.5
    private static final double MAX_JERK = 1.0;//0.25
    //region start Values NO Rotator Arm
    /*
    private static final double lowPoleHeight = 0.326;
    private static final double middlePoleHeight = 0.523;
    private static final double highPoleHeight = 0.732;
    private static final double level5ConeStackHeight = 0.140;
    private static final double aboveTheCameraHeight = 0.1;
    private static final double levelToRaiseTheConeFromStack = 0.25;
     */
    //endregion
    //region start Values WITH Rotator Arm
    private static final double lowPoleHeight = 0.1;
    private static final double middlePoleHeight = 0.297;
    private static final double highPoleHeight = 0.506;
    private static final double level5ConeStackHeight = 0.140;
    private static final double aboveTheCameraHeight = 0.1;
    private static final double levelToRaiseTheConeFromStack = 0.25;
    //endregion
    public static class MotorConstants {
        public double kV, kB, kStatic;
    }
    enum LinearSlideStates{
        Moving,
        Idle
    }
    public LinearSlideStates LinearSlideMode;
    public static final MotorConstants MOTOR_CONSTANTS = calculateMotorConstants(32.67256356,2.43,0,12);
    public static final double MOTOR_ANGULAR_MOMENT_OF_INERTIA = 0;
    public static final double LIFT_MASS = 0.5;
    private final VoltageRegulator regulator;
    private MotionProfile profile;
    private final ElapsedTime profileTimer;
    private PIDFController controller;
    public boolean isMotionProfillingBeingUsed = true;
    private boolean suppressMotionProfiling = true;
    public LinearSlidePIDWithVelocity(HardwareMap hardwareMap) {
        Realizer.realize(this, hardwareMap);
        controller = new PIDFController(new PIDCoefficients(10.0, 0, 0),//1.0,0.0,0.08
                MOTOR_CONSTANTS.kB / (diameterOfSpool/2),
                (MOTOR_ANGULAR_MOMENT_OF_INERTIA + LIFT_MASS*Math.pow((diameterOfSpool/2),2)) / (diameterOfSpool/2),
                MOTOR_CONSTANTS.kStatic,
                (x,dx) -> LIFT_MASS * 9.8066  * (diameterOfSpool/2)

                );
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Lift.setDirection(DcMotorEx.Direction.REVERSE);
        regulator = new VoltageRegulator(hardwareMap);
        profile = new MotionProfile(Collections.singletonList(new MotionSegment(new MotionState(0,0),0)));
        profileTimer = new ElapsedTime();
    }
    public boolean isBusy()
    {
        return LinearSlideMode != LinearSlideStates.Idle;
    }
    private void moveTo(double position){
        isMotionProfillingBeingUsed = !suppressMotionProfiling;
        LinearSlideMode = LinearSlideStates.Moving;

        if(isMotionProfillingBeingUsed) {
            profile = MotionProfileGenerator.generateSimpleMotionProfile(
                    new MotionState(getPosition(), getVelocity()),
                    new MotionState(position, 0),
                    MAX_VEL, MAX_ACCEL, MAX_JERK
            );
            profileTimer.reset();
            controller.reset();
        }
        else
        {
            setTargetPosition(position);
        }
    }
    public void moveToLowPole()
    {
        this.moveTo(lowPoleHeight);
    }
    public void moveToMiddlePole()
    {
        this.moveTo(middlePoleHeight);
    }
    public void moveToHighPole()
    {
        this.moveTo(highPoleHeight);
    }
    public void moveToLevel5ConeStack(){this.moveTo(level5ConeStackHeight);}
    public void moveToLevelToRaiseTheConeFromStack(){this.moveTo(levelToRaiseTheConeFromStack);}
    public void moveToAboveTheCameraHeight(){this.moveTo(aboveTheCameraHeight);}
    public double getPosition() {return metersPerTick * Lift.getCurrentPosition();}
    public double getVelocity() {return metersPerTick * Lift.getVelocity();}
    public void setTargetPosition(double height) {
        isMotionProfillingBeingUsed = false;
        if(height<=maxHeight || height>=minHeight) {
            controller.setTargetPosition(height);
        }

    }
    public void goToZero()
    {
        moveTo(0);
        //Lift.setPower(0.0);
    }

    public boolean isLiftAboveCameraHeight()
    {
        return this.getCurrentPosition() >= aboveTheCameraHeight;
    }
    public static MotorConstants calculateMotorConstants(double freeSpeed, double stallTorque, double startupVoltage, double nominalVoltage){
        MotorConstants ret = new MotorConstants();
        ret.kV = stallTorque/ nominalVoltage;
        ret.kStatic = ret.kV * startupVoltage;
        ret.kB = (stallTorque-ret.kStatic)/freeSpeed;
        return ret;

    }
    public double getCurrentPosition() {
        return metersPerTick * Lift.getCurrentPosition();
    }

    public double getCurrentVelocity() {
        return metersPerTick * Lift.getVelocity();
    }

    public void update() {
        if(isMotionProfillingBeingUsed) {
            MotionState targetState = profile.get(profileTimer.seconds());
            controller.setTargetPosition(targetState.getX());
            controller.setTargetVelocity(targetState.getV());
            controller.setTargetAcceleration(targetState.getA());
            double currentPosition = getCurrentPosition();
            double currentVelocity = getCurrentVelocity();
            double requestedTorque = controller.update(currentPosition, currentVelocity);
            double power = regulator.getRegulatedOutput(requestedTorque / MOTOR_CONSTANTS.kV);
            if (currentPosition > maxHeight) {
                power = Math.min(power, 0);
            } else if (currentPosition < minHeight) {
                power = Math.max(power, 0);
            }
            //adjust slide state
            //if(Math.abs(getCurrentPosition() - targetState.getX())<= 0.02)
            if (Math.abs(getCurrentPosition() - profile.end().getX()) <= 0.03) {
                LinearSlideMode = LinearSlideStates.Idle;
            }
            if(!liftMagneticSwitch.getState() && profile.end().getX()==0){
                power=0;
            }

            Lift.setPower(power);
        }
        else
        {
            double currentPosition = getCurrentPosition();
            double currentVelocity = getCurrentVelocity();
            double powerToApply = controller.update(currentPosition, currentVelocity);
            if(currentPosition > maxHeight){
                powerToApply = Math.min(powerToApply, 0);
            }
            else if(currentPosition < minHeight){
                powerToApply = Math.max(powerToApply,0);
            }
            if (Math.abs(getCurrentPosition() - controller.getTargetPosition()) <= 0.03) {
                LinearSlideMode = LinearSlideStates.Idle;
            }
            if(!liftMagneticSwitch.getState() && controller.getTargetPosition()==0){
                powerToApply=0;
            }
            Lift.setPower(powerToApply);

        }
    }

}
