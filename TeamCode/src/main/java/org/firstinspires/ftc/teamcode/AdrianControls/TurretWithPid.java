package org.firstinspires.ftc.teamcode.AdrianControls;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
@Config
public class TurretWithPid {

    @Hardware(name="leftEncoder")
    public DcMotorEx turretMotor;
    @Hardware(name = "MagneticLimit")
    public DigitalChannel magneticLimitSwitch;
    @Hardware(name = "TurretPotentiometer")
    public AnalogInput turretPotentiometer;
    private boolean magneticNotTouched;
    private double currentVoltageOfTurretPotentiometer;
    private double powerToApplyRight = 0.35;
    private double powerToApplyLeft = -0.35;
    private double powerToApplyRightToGoHome = 0.45;
    private double powerToApplyLeftToGoHome = -0.45;
    private double powerToApply = 0.0;
    private boolean goHomeRequested = false;
    private double turnRightVoltageValue = 1.356;
    private double turnLeftVoltageValue = 0.60;
    private double goHomeVoltageValue = 0.999;
    private double minPotentiometerValue = 0.58;
    private double maxPotentiometerValue = 1.37;

    private double turretDeadBandForHomeForInitialCheck = 0.03; //Og is 0.05
    private double turretPotValueOrigBefore = 0.0;
    private double turretDeadBandForHome = 0.0;
    private PIDFController controller;
    public static double kP=4.5; // 4.5 OG
    public static double kI =0;
    public static double kD =0;
    public static double maxPowerToApply = 0.6;


    /*
    90right = 1.3689  V

Center/Home = 0.999 V

90left = 0.61 V
     */
    public enum TurretStates
    {
        Idle,
        Right,
        Left,
        Home,
        Teleop
    }


    public TurretStates TurretMode;
    public TurretWithPid(HardwareMap hardwareMap) {
        Realizer.realize(this, hardwareMap);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        controller = new PIDFController(new PIDCoefficients(kP,kI,kD));
    }

    public boolean isBusy()
    {
        return TurretMode != TurretWithPid.TurretStates.Idle;
    }
    public double getPosition(){return  turretPotentiometer.getVoltage();}
    public void setTargetPosition(double position){controller.setTargetPosition(position);}
    public double getVelocity(){return turretMotor.getVelocity();}



    public double getTurretPotentiometerVoltage()
    {
        currentVoltageOfTurretPotentiometer = (float)turretPotentiometer.getVoltage();
        return currentVoltageOfTurretPotentiometer;
    }

    public void turretGoHome()
    {
        setTargetPosition(goHomeVoltageValue);
        TurretMode = TurretStates.Home;
    }
    public void turretGoLeft()
    {
        setTargetPosition(turnLeftVoltageValue);
        TurretMode = TurretStates.Left;
    }
    public void turretGoRight()
    {
        setTargetPosition(turnRightVoltageValue);
        TurretMode = TurretStates.Right;
    }
    public void turretGoHomeWithVoltage()
    {
        if(getTurretPotentiometerVoltage() < goHomeVoltageValue)
        {
            powerToApply = powerToApplyRightToGoHome;
        }
        else
        {
            powerToApply = powerToApplyLeftToGoHome;
        }
        if(magneticLimitSwitch.getState() && Math.abs(getTurretPotentiometerVoltage() - goHomeVoltageValue) > turretDeadBandForHomeForInitialCheck)
        {
            TurretMode = TurretStates.Home;
            //turretPotValueOrigBefore = getTurretPotentiometerVoltage();
            if(getTurretPotentiometerVoltage() < goHomeVoltageValue)
            {
                turretDeadBandForHome  = -1.0 * turretDeadBandForHomeForInitialCheck;
            }
            else
            {
                turretDeadBandForHome = 1.0 * turretDeadBandForHomeForInitialCheck;
            }
        }
        else
        {
            TurretMode = TurretStates.Idle;
            powerToApply = 0.0;
        }
    }

    public void turretGoLeftWithVoltage()
    {
        TurretMode = TurretStates.Left;
        if(getTurretPotentiometerVoltage()>turnLeftVoltageValue)
        {
            powerToApply = powerToApplyLeft;
        }
        else
        {
            TurretMode = TurretStates.Idle;
            powerToApply = 0.0;
        }
    }
    public void turretGoRightWithVoltage()
    {
        TurretMode = TurretStates.Right;
        if(getTurretPotentiometerVoltage()<turnRightVoltageValue)
        {
            powerToApply = powerToApplyRight;
        }
        else
        {
            TurretMode = TurretStates.Idle;
            powerToApply = 0.0;
        }
    }

    public void update() {
        double currentPosition = getPosition();
        double currentVelocity = getVelocity();
        powerToApply = controller.update(currentPosition,currentVelocity);
        if(Math.abs(powerToApply) >= maxPowerToApply)
        {
            if(powerToApply<0)
            {
                powerToApply = -1*maxPowerToApply;
            }
            else
            {
                powerToApply = maxPowerToApply;
            }
        }
        if(currentPosition > maxPotentiometerValue){
            powerToApply = Math.min(powerToApply, 0);
        }
        else if(currentPosition < minPotentiometerValue){
            powerToApply = Math.max(powerToApply,0);
        }

        if(Math.abs(getPosition() - controller.getTargetPosition()) <= turretDeadBandForHomeForInitialCheck)
        {
            powerToApply = 0;
            TurretMode = TurretStates.Idle;
        }
        if(TurretMode != TurretStates.Teleop)
        {
            turretMotor.setPower(powerToApply);
        }
        /*
        if(TurretMode == TurretStates.Home)
        {
            if( ((powerToApply > 0) && (getTurretPotentiometerVoltage() - goHomeVoltageValue) > turretDeadBandForHome)
            || ((powerToApply < 0) && (getTurretPotentiometerVoltage() - goHomeVoltageValue) < turretDeadBandForHome)
            )
            {
                powerToApply = 0;
                turretDeadBandForHome = 0.0;
                TurretMode = TurretStates.Idle;
            }
        }
        if(TurretMode == TurretStates.Left)
        {
            if(getTurretPotentiometerVoltage() < turnLeftVoltageValue )
            {
                powerToApply = 0.0;
                TurretMode = TurretStates.Idle;
            }

        }

        if(TurretMode == TurretStates.Right)
        {
            if(getTurretPotentiometerVoltage() > turnRightVoltageValue )
            {
                powerToApply = 0.0;
                TurretMode = TurretStates.Idle;
            }

        }
        if(TurretMode != TurretStates.Teleop ) {
            turretMotor.setPower(powerToApply);
        }
        */
    }


}
