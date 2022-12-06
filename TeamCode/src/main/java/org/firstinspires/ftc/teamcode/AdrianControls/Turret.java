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
    private double powerToApplyRight = 0.45;
    private  double powerToApplyLeft = -0.45;
    private double powerToApply = 0.0;
    private boolean goHomeRequested = false;
    private double turnRightVoltageValue = 1.3189;
    private double turnLeftVoltageValue = 0.66;
    private double goHomeVoltageValue = 0.999;

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
            powerToApply = powerToApplyRight;
        }
        else
        {
            powerToApply = powerToApplyLeft;
        }
        if(magneticLimitSwitch.getState() && Math.abs(getTurretPotentiometerVoltage() - goHomeVoltageValue) > 0.05)
        {
            TurretMode = TurretStates.Home;
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
        if(TurretMode == TurretStates.Home)
        {
            if((!magneticLimitSwitch.getState()) || Math.abs(getTurretPotentiometerVoltage() - goHomeVoltageValue) < 0.05 )
            {
                powerToApply = 0;
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
