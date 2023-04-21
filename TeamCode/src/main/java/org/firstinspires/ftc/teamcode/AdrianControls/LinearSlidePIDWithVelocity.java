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

import java.util.Collections;

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
    private double currentPositionForClaw = 0.0;
    private double lowerCalculatedHeight = 0.0;
    public double subtractionForLowerCalcualtedHeight = 0.15;
    public double subtractionForLowerCalcualtedHeightFast = 0.09;
    private static final double lowPoleHeight = 0.1;
    private static final double middlePoleHeight = 0.308; //0.297 og

    private static final double middlePoleHeightForAutoFirstPole = 0.33; //0.308 og

    private static final double highPoleHeight = 0.521; //506 og
    private static final double lowPoleHeightTeleop = 0.326;
    private static final double middlePoleHeightTeleop = 0.551;
    private static final double highPoleHeightTeleop = 0.745;
    private static final double level5ConeStackHeight = 0.124;//0.119 //0.120
    private static final double level4ConeStackHeight = 0.096; //0.091//0.092
    private static final double level3ConeStackHeight = 0.067; //0.062// 0.063
    private static final double level2ConeStackHeight = 0.030; //0.025 //0.028
    private static final double aboveTheCameraHeight = 0.1;
    private static final double levelToRaiseTheConeFromStack = 0.24; // 0.25 was orig.
    private static final double middlePoleHeightForTurretAutonomous = 0.38; //0.39 //0.35 // 0.308// 0.42 for 45 deg rotator arm; //0.36 for 90 deg rotator arm
    public double subtractionForLowerCalcualtedHeightForTurretAutonomous = 0.12; //0.2, 0.15
    public boolean isTurretUsed = false;

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
    public LinearSlidePIDWithVelocity(HardwareMap hardwareMap, boolean isTurretUsed) {
        this.isTurretUsed = isTurretUsed;
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
    public void moveToBelowOriginalLevel(double subtractForLower)
    {
        currentPositionForClaw = this.getPosition();
        lowerCalculatedHeight = currentPositionForClaw - subtractForLower;
        this.moveTo(lowerCalculatedHeight);
    }
    public void moveToOriginalLevel()
    {
        this.moveTo(currentPositionForClaw);
    }
    public void resetOrignalLevelValues()
    {
        currentPositionForClaw = 0.0;
        lowerCalculatedHeight = 0.0;
    }

    public void moveToLowPole()
    {
        this.moveTo(lowPoleHeight);
    }
    public void moveToMiddlePole()
    {
        if(isTurretUsed)
        {
            this.moveTo(middlePoleHeightForTurretAutonomous);

        }
        else {
            this.moveTo(middlePoleHeight);
        }
    }
    public void moveToMiddlePoleForFirstPole()
    {
            this.moveTo(middlePoleHeight); //middlePoleHeight
    }
    public void moveToHighPole()
    {
        this.moveTo(highPoleHeight);
    }
    public void moveToLowPoleTeleop()
    {
        this.moveTo(lowPoleHeightTeleop);
    }
    public void moveToMiddlePoleTeleop()
    {
        this.moveTo(middlePoleHeightTeleop);
    }
    public void moveToHighPoleTeleop()
    {
        this.moveTo(highPoleHeightTeleop);
    }
    public void moveToLevel5ConeStack(){this.moveTo(level5ConeStackHeight);}
    public void moveToLevel4ConeStack(){this.moveTo(level4ConeStackHeight);}
    public void moveToLevel3ConeStack(){this.moveTo(level3ConeStackHeight);}
    public void moveToLevel2ConeStack(){this.moveTo(level2ConeStackHeight);}
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
