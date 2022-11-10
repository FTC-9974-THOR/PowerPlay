package org.firstinspires.ftc.teamcode.AdrianControls;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;

public class LinearSlidePID {

    @Hardware(name="SlideMotor")
    public DcMotorEx Lift;
    private static final double diameterOfSpool = 0.03565; // in meters- diameter.
    private static final double metersPerTick = (Math.PI * diameterOfSpool) / (7 * 4 * 13.7);
    private static final double minHeight = 0; // meters
    private static final double maxHeight = 0.5; // also meters
    private PIDFController controller;

    public LinearSlidePID(HardwareMap hardwareMap) {
        Realizer.realize(this, hardwareMap);
        controller = new PIDFController(new PIDCoefficients(2, 0, 0));
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Lift.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void setTargetPosition(double height) {
        controller.setTargetPosition(height);
    }

    public double getCurrentPosition() {
        return metersPerTick * Lift.getCurrentPosition();
    }

    public double getCurrentVelocity() {
        return metersPerTick * Lift.getVelocity();
    }

    public void update() {
        double currentPosition = getCurrentPosition();
        double currentVelocity = getCurrentVelocity();
        double powerToApply = controller.update(currentPosition, currentVelocity);
        if(currentPosition > maxHeight){
            powerToApply = Math.min(powerToApply, 0);
        }
        else if(currentPosition < minHeight){
            powerToApply = Math.max(powerToApply,0);
        }
        Lift.setPower(powerToApply);


    }

}
