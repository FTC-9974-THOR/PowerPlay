package org.firstinspires.ftc.teamcode.AdrianControls;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionSegment;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ftc9974.thorcore.control.VoltageRegulator;
import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;

import java.util.Collections;

public class MagneticSwitch {
    @Hardware(name = "MagneticLimit")
    public DigitalChannel magneticLimitSwitch;
    @Hardware(name = "TurretMotor")
    public DcMotorEx turretMotor;
// We are getting the state of the magneticLimitSwitch(True means it's not touched)
    private boolean magneticNotTouched = magneticLimitSwitch.getState();
    public MagneticSwitch(HardwareMap hardwareMap) {
        Realizer.realize(this, hardwareMap);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }
// This is a function that is called in autonomous, but could be called in Teleop too
//Basically, I am taking the position you want to go to, either one or two,  and using that to determine the power of the motor.
    public void turnTheTurretCorrectly(int positionOne, int positionTwo) {
        if (positionOne > 0) {
            while (magneticNotTouched == true) {
                turretMotor.setPower(0.5);
            }
            turretMotor.setPower(0.0);

        }
        if (positionTwo > 0) {
            while (magneticNotTouched == true) {
                turretMotor.setPower(-0.5);
            }
            turretMotor.setPower(0.0);

        }
    }
}
