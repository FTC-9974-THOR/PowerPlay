//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//
package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Image;

import java.io.IOException;
import java.util.BitSet;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.ftc9974.thorcore.robot.sensors.USBWebcam;
import org.firstinspires.ftc.teamcode.AdrianControls.VuforiaStuff2023;
@Disabled
@TeleOp(
        name = "TeleOpCameraTester",
        group = "ThorCore Samples"
)
public class TeleOpCameraTester extends OpMode {
    private USBWebcam webcam;

    public TeleOpCameraTester() {
    }

    public void init() {
        try {
            this.webcam = new USBWebcam("Webcam 1", this.hardwareMap);
            this.webcam.startStreaming();
        } catch (IOException var2) {
            RobotLog.ee("SampleUSBWebcam", var2, "IOException initializing webcam: %s", new Object[]{var2.getMessage()});
            this.telemetry.addLine("Init failed.");
        }

    }

    public void loop() {
        //CameraFrame frame = this.webcam.getLastFrame();
        //frame.releaseRef();
    }

    public void stop() {
        this.webcam.shutdown();
    }
}
