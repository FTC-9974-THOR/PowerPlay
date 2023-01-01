package org.firstinspires.ftc.teamcode.AdrianControls;

import android.graphics.Rect;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.ftc9974.thorcore.robot.sensors.USBWebcamBase;
import org.ftc9974.thorcore.vision.NEONVision;
import org.ftc9974.thorcore.vision.NativeImageByteBuffer;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicBoolean;


public class NeonVision2023 extends USBWebcamBase {
    public enum sleeveSignal {
        ONEDOT, TWODOTS, THREEDOTS, NONE;
    }
    public class sleeveSignalDetectedData{
        public sleeveSignal sleeveSignalDetected = sleeveSignal.NONE;
        public double oneDotCount;
        public double twoDotsCount;
        public double threeDotsCount;
    }
    private NativeImageByteBuffer mask;
    private sleeveSignalDetectedData sleeveSignalDetectedDataReturn = new sleeveSignalDetectedData();
    private final Object lock = new Object();
    AtomicBoolean hasConstructorFinished = new AtomicBoolean(false);

    public sleeveSignalDetectedData getSleeveSignalDetectedDataReturn()
    {
        synchronized (lock){
        return sleeveSignalDetectedDataReturn;
        }
     }
    public NeonVision2023(String name, HardwareMap hw) throws IOException {

        super(name, hw);
        mask = new NativeImageByteBuffer(getFrameSize().getWidth(),getFrameSize().getHeight());
        mask.drawFilledRectangle(new Rect(0,getFrameSize().getHeight()/2,getFrameSize().getWidth(),getFrameSize().getHeight()),(byte)1);
        hasConstructorFinished.set(true);
    }



    @Override
    protected void onNewFrame(CameraFrame frame) {
        if(!hasConstructorFinished.get())
            return;

        long purpleValue = NEONVision.processYUY2WithMask(
                frame.getImageBuffer(),
                frame.getImageSize(),
                NEONVision.yuvColorLong(0xff,0xff,0xff),
                NEONVision.yuvColorLong(0x00,0x8c,0x8c),
                mask.getPointer()
        );
        long greenValue = NEONVision.processYUY2WithMaskForDisplay(
                frame.getImageBuffer(),
                frame.getImageSize(),
                NEONVision.yuvColorLong(0xff,0x84,0x65),
                NEONVision.yuvColorLong(0x92,0x5f,0x3d),
                mask.getPointer()
                );
        /*
        long yellowValue = NEONVision.processYUY2(
                frame.getImageBuffer(),
                frame.getImageSize(),
                NEONVision.yuvColorLong(0xef,0x72,0x87),
                NEONVision.yuvColorLong(0xca,0x53,0x69)
        );

         */

        synchronized (lock)
        {
            float greenRatio = (float)greenValue/(greenValue+purpleValue);
            float purpleRatio = (float)purpleValue/(greenValue+purpleValue);
            sleeveSignalDetectedDataReturn.oneDotCount = greenValue;
            sleeveSignalDetectedDataReturn.twoDotsCount = greenRatio+purpleRatio;
            sleeveSignalDetectedDataReturn.threeDotsCount = purpleValue;

            if(greenRatio> 0.7)
                sleeveSignalDetectedDataReturn.sleeveSignalDetected = sleeveSignal.ONEDOT;
            else if(purpleRatio > 0.7)
                sleeveSignalDetectedDataReturn.sleeveSignalDetected = sleeveSignal.THREEDOTS;
            else
                sleeveSignalDetectedDataReturn.sleeveSignalDetected = sleeveSignal.TWODOTS;
/*
            if(greenValue > yellowValue && greenValue > purpleValue)
            {
                sleeveSignalDetectedDataReturn.sleeveSignalDetected = sleeveSignal.ONEDOT;
            }
            if(yellowValue > greenValue && yellowValue > purpleValue)
            {
                sleeveSignalDetectedDataReturn.sleeveSignalDetected = sleeveSignal.TWODOTS;
            }
            if(purpleValue > greenValue && purpleValue > yellowValue)
            {
                sleeveSignalDetectedDataReturn.sleeveSignalDetected = sleeveSignal.THREEDOTS;
            }

 */
        }
    }
}
