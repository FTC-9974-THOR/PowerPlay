package org.firstinspires.ftc.teamcode.AdrianControls;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;

public class Turret {

    @Hardware(name="leftEncoder")
    public DcMotorEx turretMotor;
    @Hardware(name = "MagneticLimit")
    public DigitalChannel magneticLimitSwitch;
    @Hardware(name = "TurretPotentiometer")
    public AnalogInput turretPotentiometer;
    private boolean magneticNotTouched;
    private double currentVoltageOfTurretPotentiometer;
    private double powerToApplyRight = 0.55; //0.45
    private double powerToApplyLeft = -0.55; //0.45
    private double powerToApplyRightToGoHome = 0.65; //0.55
    private double powerToApplyLeftToGoHome = -0.65; //0.55
    private double powerToApply = 0.0;
    private boolean goHomeRequested = false;
    private double turnRightVoltageValue = 1.356;
    private double turnLeftVoltageValue = 0.60;
    private double goHomeVoltageValue = 0.999;
    private double turretDeadBandForHomeForInitialCheck = 0.02; //Og is 0.05
    private double turretPotValueOrigBefore = 0.0;
    private double turretDeadBandForHome = 0.0;
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
    public Turret(HardwareMap hardwareMap) {
        Realizer.realize(this, hardwareMap);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPowerToApply(double power) {
        powerToApply = power;
    }

    public boolean IsTurretHome() {
        return !magneticLimitSwitch.getState();
    }
    public boolean isBusy()
    {
        return TurretMode != Turret.TurretStates.Idle;
    }

    public double getTurretPotentiometerVoltage()
    {
        currentVoltageOfTurretPotentiometer = (float)turretPotentiometer.getVoltage();
        return currentVoltageOfTurretPotentiometer;
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
/*
    public void turretGoHome(int teamcolor)
    {
        if(teamcolor == -1)
        {
            powerToApply = powerToApplyRight;
        }
        if(teamcolor == 1)
        {
            powerToApply = powerToApplyLeft;
        }

        if(magneticLimitSwitch.getState())
        {
            TurretMode = TurretStates.Home;
        }
        else
        {
            TurretMode = TurretStates.Idle;
        }
    }
*/
    public void update() {
      /*
        if(TurretMode == TurretStates.Home)
        {
            if((!magneticLimitSwitch.getState()) || Math.abs(getTurretPotentiometerVoltage() - goHomeVoltageValue) < turretDeadBandForHome )
            {
                powerToApply = 0;
                TurretMode = TurretStates.Idle;
            }
        }

*/
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
    }


}
