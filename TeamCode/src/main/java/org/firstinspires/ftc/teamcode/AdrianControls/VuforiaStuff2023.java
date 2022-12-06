package org.firstinspires.ftc.teamcode.AdrianControls;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Environment;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

import static android.graphics.Bitmap.createBitmap;
import static android.graphics.Bitmap.createScaledBitmap;

public class VuforiaStuff2023 {

    VuforiaLocalizer vuforia;

    public VuforiaStuff2023(VuforiaLocalizer vuforia) {
        this.vuforia = vuforia;
    }

    public enum sleeveSignal {
        ONEDOT, TWODOTS, THREEDOTS;
    }

    public class sleeveSignalDetectedData{
        public sleeveSignal sleeveSignalDetected;
        public double oneDotCount;
        public double twoDotsCount;
        public double threeDotsCount;
    }
    public sleeveSignalDetectedData vuforiascan(boolean saveBitmaps, boolean red, boolean RGB,int sideWeStart ) {
        //sideWeStart;//1=Left -1= Right
        Image rgbImage = null;
        int rgbTries = 0;
        /*
        double colorcountL = 0;
        double colorcountC = 0;
        double colorcountR = 0;
        */
        double oneDotCountInternal = 1;
        double twoDotsCountInternal = 1;
        double threeDotsCountInternal = 1;
        //ForRedDectection Thresholds
        int RedDetectionredThreshold = 150;
        int RedDetectiongreenThreshold = 190;
        int RedDetectionblueThreshold = 160;
        //ForGreenDectection Thresholds
        int GreenDetectionredThreshold = 150;
       // int GreenDetectiongreenThreshold = 100;
        int GreenDetectiongreenThreshold = 110;

        int GreenDetectionblueThreshold = 150;
        //ForBlueDectection Thresholds
        int BlueDetectionredThreshold = 190;
        int BlueDetectiongreenThreshold = 190;
        int BlueDetectionblueThreshold = 160;
        //ForYellowDetection Thresholds
        int YellowDetectionredThreshold = 140;
        int YellowDetectiongreenThreshold = 130;
        int YellowDetectionblueThreshold = 150;
        //ForPurpleDetection Thresholds
        int PurpleDetectionredThreshold = 90;
        int PurpleDetectiongreenThreshold = 150;
        int PurpleDetectionblueThreshold = 90;

        //region start Crop Variable With Turrent Design with Robot starting in One Corner
        //Crop Variables Start
        /*
        double cropStartXLeft = 64.0;
        double cropStartYLeft = 45.0;
        double cropWidthLeft = 60.0;
        double cropHeightLeft = 93.0;
        double cropStartXRight = 139.0;
        double cropStartYRight = 45.0;
        double cropWidthRight = 60.0;
        double cropHeightRight = 93.0;
        double MaxWidth=640.0;
        double MaxHeight=480.0;
        */
        //Crop Variables End
        //endregion

        //region start Crop Variable With Rotating Arm Design with Robot starting in the Middle
        //Crop Variables Start
        double cropStartXLeft = 126.0;
        double cropStartYLeft = 45.0;
        double cropWidthLeft = 60.0;
        double cropHeightLeft = 93.0;
        double cropStartXRight = 126.0;
        double cropStartYRight = 45.0;
        double cropWidthRight = 60.0;
        double cropHeightRight = 93.0;
        double MaxWidth=640.0;
        double MaxHeight=480.0;
        
        //Crop Variables End
        //endregion
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        VuforiaLocalizer.CloseableFrame closeableFrame = null;
        this.vuforia.setFrameQueueCapacity(1);
        while (rgbImage == null) {
            try {
                closeableFrame = this.vuforia.getFrameQueue().take();
                long numImages = closeableFrame.getNumImages();

                for (int i = 0; i < numImages; i++) {
                    if (closeableFrame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                        rgbImage = closeableFrame.getImage(i);
                        if (rgbImage != null) {
                            break;
                        }
                    }
                }
            } catch (InterruptedException exc) {

            } finally {
                if (closeableFrame != null) closeableFrame.close();
            }
        }

        if (rgbImage != null) {

            // copy the bitmap from the Vuforia frame
            Bitmap bitmap = Bitmap.createBitmap(rgbImage.getWidth(), rgbImage.getHeight(), Bitmap.Config.RGB_565);
            bitmap.copyPixelsFromBuffer(rgbImage.getPixels());

            String path = Environment.getExternalStorageDirectory().toString();
            String pathex = Environment.getExternalStorageDirectory().getAbsolutePath().toString();

            //String path = Environment.getExternalStoragePublicDirectory("Download").toString();
            FileOutputStream out = null;

            String bitmapName;
            String croppedBitmapName;

            if (red) {
                bitmapName = "BitmapRED.png";
                croppedBitmapName = "BitmapCroppedRED.png";
            } else {
                bitmapName = "BitmapBLUE.png";
                croppedBitmapName = "BitmapCroppedBLUE.png";
            }

            //Save bitmap to file
            if (saveBitmaps) {
                try {
                    File file = new File(path, bitmapName);
                    out = new FileOutputStream(file);
                    bitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
                } catch (Exception e) {
                    e.printStackTrace();
                } finally {
                    try {
                        if (out != null) {
                            out.flush();
                            out.close();
                        }
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
            }

            int cropStartX;
            int cropStartY;
            int cropWidth;
            int cropHeight;
            if(sideWeStart == -1)//right
            {
                cropStartX = (int) ((cropStartXRight / MaxWidth) * bitmap.getWidth());
                cropStartY = (int) ((cropStartYRight / MaxHeight) * bitmap.getHeight());
                cropWidth = (int) ((cropWidthRight / MaxWidth) * bitmap.getWidth());
                cropHeight = (int) ((cropHeightRight / MaxHeight) * bitmap.getHeight());

            }
            else //left
            {
                cropStartX = (int) ((cropStartXLeft / MaxWidth) * bitmap.getWidth());
                cropStartY = (int) ((cropStartYLeft / MaxHeight) * bitmap.getHeight());
                cropWidth = (int) ((cropWidthLeft / MaxWidth) * bitmap.getWidth());
                cropHeight = (int) ((cropHeightLeft / MaxHeight) * bitmap.getHeight());

            }

            bitmap = createBitmap(bitmap, cropStartX, cropStartY, cropWidth, cropHeight); //Cropped Bitmap to show only stones

            // Save cropped bitmap to file
            if (saveBitmaps) {
                try {
                    File file = new File(path, croppedBitmapName);
                    out = new FileOutputStream(file);
                    bitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
                } catch (Exception e) {
                    e.printStackTrace();
                } finally {
                    try {
                        if (out != null) {
                            out.flush();
                            out.close();
                        }
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
            }
            //bitmap =  createScaledBitmap(bitmap, 110, 20, true); //Compress bitmap to reduce scan time
            bitmap =  createScaledBitmap(bitmap, 40, 20, true); //Compress bitmap to reduce scan time

            int height;
            int width;
            int pixel;
            int bitmapWidth = bitmap.getWidth();
            int bitmapHeight = bitmap.getHeight();
//            int colWidth = (int) ((double) bitmapWidth / 6.0);
//            int colorLStartCol = (int) ((double) bitmapWidth * (1.0 / 6.0) - ((double) colWidth / 1.1));
//            int colorCStartCol = (int) ((double) bitmapWidth * (3.0 / 6.0) - ((double) colWidth / 2.0));
//            int colorRStartCol = (int) ((double) bitmapWidth * (5.0 / 6.0) - ((double) colWidth / 9.0));
        if(RGB){
            for (height = 0; height < bitmapHeight; ++height) {
                for (width = 0; width < bitmapWidth; ++width) {
                    pixel = bitmap.getPixel(width, height);
                    int redPixel=Color.red(pixel);
                    int greenPixel=Color.green(pixel);
                    int bluePixel=Color.blue(pixel);
                    if (redPixel > RedDetectionredThreshold && greenPixel < RedDetectiongreenThreshold && bluePixel < RedDetectionblueThreshold) {
                        oneDotCountInternal += Color.red(pixel);
                    }
                    else
                    {
                        if (greenPixel > GreenDetectiongreenThreshold && redPixel < GreenDetectionredThreshold && bluePixel < GreenDetectionblueThreshold) {
                            twoDotsCountInternal += Color.green(pixel);
                        }
                        else
                        {
                            if (bluePixel > BlueDetectionblueThreshold && greenPixel < BlueDetectiongreenThreshold && redPixel<BlueDetectionredThreshold) {
                                threeDotsCountInternal += Color.blue(pixel);
                                }
                            }
                        }

                    }

                }



            }
            else {
                for (height = 0; height < bitmapHeight; ++height) {
                    for (width = 0; width < bitmapWidth; ++width) {
                        pixel = bitmap.getPixel(width, height);
                        int redPixel=Color.red(pixel);
                        int greenPixel=Color.green(pixel);
                        int bluePixel=Color.blue(pixel);
                        if (redPixel > YellowDetectionredThreshold && greenPixel > YellowDetectiongreenThreshold && bluePixel < YellowDetectionblueThreshold) {
                            twoDotsCountInternal += Color.red(pixel);
                        }
                        else
                        {
                            if (greenPixel > GreenDetectiongreenThreshold && redPixel < GreenDetectionredThreshold && bluePixel < GreenDetectionblueThreshold) {
                                oneDotCountInternal += Color.green(pixel);
                            }
                            else
                            {
                                if (bluePixel > PurpleDetectionblueThreshold && greenPixel < PurpleDetectiongreenThreshold && redPixel>PurpleDetectionredThreshold) {
                                    threeDotsCountInternal += Color.blue(pixel);
                                }
                            }
                        }

                    }

                }



            }
        }

        double oneDotCountFinal = oneDotCountInternal;
        double twoDotsCountFinal = twoDotsCountInternal;
        double threeDotsCountFinal = threeDotsCountInternal;



        sleeveSignal signal;
        sleeveSignalDetectedData signalData = new sleeveSignalDetectedData();

        if (oneDotCountFinal > twoDotsCountFinal && oneDotCountFinal > threeDotsCountFinal) {
            signal = sleeveSignal.ONEDOT;
        } else if (twoDotsCountFinal > oneDotCountFinal && twoDotsCountFinal > threeDotsCountFinal) {
            signal = sleeveSignal.TWODOTS;
        } else {
            signal = sleeveSignal.THREEDOTS;
        }
        signalData.oneDotCount = oneDotCountFinal;
        signalData.twoDotsCount = twoDotsCountFinal;
        signalData.threeDotsCount = threeDotsCountFinal;
        signalData.sleeveSignalDetected = signal;
        return signalData;

    }
}