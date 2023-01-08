package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.ftc9974.thorcore.robot.sensors.USBWebcam;
import org.ftc9974.thorcore.robot.sensors.USBWebcamBase;
import org.ftc9974.thorcore.util.BooleanEdgeDetector;
import org.ftc9974.thorcore.util.MathUtilities;
import org.ftc9974.thorcore.vision.NEONVision;

import java.io.IOException;

@TeleOp(name = "NEONVision Color Test")
@Disabled
public class NEONVisionColorTestOpMode extends LinearOpMode {

    private static class YUVColor {
        double y, u, v;

        YUVColor(int y, int u, int v) {
            this.y = y;
            this.u = u;
            this.v = v;
        }

        long toLong() {
            return NEONVision.yuvColorLong((int) y, (int) u, (int) v);
        }

        @NonNull
        @Override
        public String toString() {
            return String.format("y: 0x%02x u: 0x%02x v: 0x%02x", (int) y, (int) u, (int) v);
        }
    }

    private class NEONStreamer extends USBWebcamBase {
        NEONStreamer(String name, HardwareMap hw) throws IOException {
            super(name, hw);
        }

        @Override
        protected void onNewFrame(CameraFrame frame) {
            synchronized (NEONVisionColorTestOpMode.this.lock) {
                match = NEONVision.processYUY2ForDisplay(
                        frame.getImageBuffer(), frame.getImageSize(),
                        NEONVisionColorTestOpMode.this.high.toLong(),
                        NEONVisionColorTestOpMode.this.low.toLong()
                );
            }
        }
    }

    private enum Adjustment {
        HIGH_Y,
        HIGH_U,
        HIGH_V,
        LOW_Y,
        LOW_U,
        LOW_V
    }

    private static final double ADJUSTMENT_SPEED = 0x20;

    protected YUVColor high, low;
    protected long match;
    protected final Object lock = new Object();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(10);

        high = new YUVColor(0xff, 0xff, 0xff);
        low = new YUVColor(0x00, 0x00, 0x00);
        Adjustment currentAdjustment = Adjustment.HIGH_Y;

        BooleanEdgeDetector nextAdjustmentDetector = new BooleanEdgeDetector(false);

        if (!NEONVision.supportsNeonAcceleration()) {
            throw new UnsupportedOperationException("NEONVision is not supported on this processor architecture");
        }

        NEONStreamer streamer;
        try {
            streamer = new NEONStreamer("Webcam 1", hardwareMap);
        } catch (IOException e) {
            e.printStackTrace();
            RobotLog.ee("NEONVisionColorTest", e, "IOException initializing camera");
            requestOpModeStop();
            return;
        }

        streamer.startStreaming();

        ElapsedTime timer = new ElapsedTime();
        while (!isStopRequested() && !isStarted()) {
            double deltaT = timer.seconds();
            timer.reset();

            nextAdjustmentDetector.update(gamepad1.a);
            if (nextAdjustmentDetector.isRising()) {
                // move to the next value in the enum, wrapping around at the end
                currentAdjustment = Adjustment.values()[(currentAdjustment.ordinal() + 1) % Adjustment.values().length];
            }

            telemetry.addData("Current Adjustment", currentAdjustment);

            double deltaAdjustment = -gamepad1.right_stick_y * ADJUSTMENT_SPEED * deltaT;
            telemetry.addData("Delta", deltaAdjustment);
            synchronized (lock) {
                telemetry.addData("Match Amount", match);
                telemetry.addData("High", high.toString());
                telemetry.addData("Low", low.toString());
                switch (currentAdjustment) {
                    case HIGH_Y:
                        high.y = MathUtilities.constrain(high.y + deltaAdjustment, 0x00, 0xff);
                        break;
                    case HIGH_U:
                        high.u = MathUtilities.constrain(high.u + deltaAdjustment, 0x00, 0xff);
                        break;
                    case HIGH_V:
                        high.v = MathUtilities.constrain(high.v + deltaAdjustment, 0x00, 0xff);
                        break;
                    case LOW_Y:
                        low.y = MathUtilities.constrain(low.y + deltaAdjustment, 0x00, 0xff);
                        break;
                    case LOW_U:
                        low.u = MathUtilities.constrain(low.u + deltaAdjustment, 0x00, 0xff);
                        break;
                    case LOW_V:
                        low.v = MathUtilities.constrain(low.v + deltaAdjustment, 0x00, 0xff);
                        break;
                }
            }

            telemetry.update();

            sleep(10);
        }

        streamer.stopStreaming();
        streamer.shutdown();

        if (isStopRequested()) return;
        telemetry.addData("High", high.toString());
        telemetry.addData("Low", low.toString());
        telemetry.update();
        while (!isStopRequested());
    }
}
