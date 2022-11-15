package org.firstinspires.ftc.teamcode.AdrianControls;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
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
    private boolean magneticNotTouched;
    private double powerToApply = 0;
    private double goHomePowerToApplyRight = 0.35;
    private  double goHomePowerToApplyLeft = -0.35;
    private double goHomePowerToApply = 0.0;
    private boolean goHomeRequested = false;
    private enum TurretStates
    {
        Idle,
        Moving
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


    public void turretGoHome(int teamcolor)
    {
        if(teamcolor == -1)
        {
            goHomePowerToApply = goHomePowerToApplyRight;
        }
        if(teamcolor == 1)
        {
            goHomePowerToApply = goHomePowerToApplyLeft;
        }

        if(magneticLimitSwitch.getState())
        {
            goHomeRequested = true;
            TurretMode = TurretStates.Moving;
        }
        else
        {
            goHomeRequested = false;
            TurretMode = TurretStates.Idle;
        }
    }

    public void update() {
        if(goHomeRequested)
        {
            if(magneticLimitSwitch.getState())
            {
                powerToApply = goHomePowerToApply;

            }
            else
            {
                powerToApply = 0;
                goHomeRequested = false;
                TurretMode = TurretStates.Idle;
            }
        }
        turretMotor.setPower(powerToApply);
    }


}
