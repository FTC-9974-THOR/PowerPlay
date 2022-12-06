package org.firstinspires.ftc.teamcode.AdrianControls;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.ftc9974.thorcore.meta.Realizer;
import org.ftc9974.thorcore.meta.annotation.Hardware;
import org.ftc9974.thorcore.util.MathUtilities;

public class Claw {
    public static final double CLAW_OPEN = MathUtilities.map(1100, 500, 2500, 0,1),
            CLAW_CLOSED = MathUtilities.map(1485, 500, 2500, 0,1);
    @Hardware(name = "ClawServo")

    public Servo clawServo;
// We are getting the state of the magneticLimitSwitch(True means it's not touched)
    public Claw(HardwareMap hardwareMap) {
        Realizer.realize(this, hardwareMap);
    }
// This is a function that is called in autonomous, but could be called in Teleop too
//Basically, I am taking the position you want to go to, either one or two, and using that to determine the power of the motor.
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

}

