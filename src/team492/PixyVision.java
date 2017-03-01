/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team492;

import org.opencv.core.Rect;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import frclib.FrcPixyCam;
import trclib.TrcDbgTrace;
import trclib.TrcPixyCam.ObjectBlock;

public class PixyVision
{
    private static final String moduleName = "PixyVision";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    public class TargetInfo
    {
        public Rect rect;
        public double distance;
        public double angle;

        public TargetInfo(Rect rect, double distance, double angle)
        {
            this.rect = rect;
            this.distance = distance;
            this.angle = angle;
        }   //TargetInfo

    }   //class TargetInfo

    public enum Orientation
    {
        NORMAL_LANDSCAPE,
        CLOCKWISE_PORTRAIT,
        ANTICLOCKWISE_PORTRAIT,
        UPSIDEDOWN_LANDSCAPE
    }   //enum Orientation

    private static final double PIXY_DISTANCE_SCALE = 2300.0;   //DistanceInInches*targetWidthdInPixels
    private static final double TARGET_WIDTH_INCHES = 10.0;

    private FrcPixyCam pixyCamera;
    private int signature;
    private Orientation orientation;

    private void commonInit(int signature, int brightness, Orientation orientation)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        this.signature = signature;
        this.orientation = orientation;
        pixyCamera.setBrightness((byte)brightness);
    }   //commonInit

    public PixyVision(
        final String instanceName, int signature, int brightness, Orientation orientation,
        I2C.Port port, int i2cAddress)
    {
        pixyCamera = new FrcPixyCam(instanceName, port, i2cAddress);
        commonInit(signature, brightness, orientation);
    }   //PixyVision

    public PixyVision(
        final String instanceName, int signature, int brightness, Orientation orientation, SerialPort.Port port)
    {
        pixyCamera = new FrcPixyCam(instanceName, port);
        commonInit(signature, brightness, orientation);
    }   //PixyVision

    public void setEnabled(boolean enabled)
    {
        pixyCamera.setEnabled(enabled);
    }   //setEnabled

    public boolean isEnabled()
    {
        return pixyCamera.isEnabled();
    }   //isEnabled

    /**
     * This method returns the rectangle of the last detected target.
     *
     * @return rectangle of last detected target.
     */
    private Rect getTargetRect()
    {
        Rect targetRect = null;
        ObjectBlock[] detectedObjects = pixyCamera.getDetectedObjects();

        if (debugEnabled)
        {
            dbgTrace.traceInfo(moduleName, "%s object(s) found",
                detectedObjects != null? "" + detectedObjects.length: "null");
        }

        if (detectedObjects != null && detectedObjects.length >= 2)
        {
            ObjectBlock[] targets = new ObjectBlock[2];
            int j = 0;

            for (int i = 0; i < detectedObjects.length && j < targets.length; i++)
            {
                if (debugEnabled)
                {
                    dbgTrace.traceInfo(moduleName, "[%d] %s", i, detectedObjects[i].toString());
                }

                if (signature == detectedObjects[i].signature)
                {
                    targets[j] = detectedObjects[i];
                    j++;
                }
            }

            if (j == 2)
            {
                int targetCenterX = (targets[0].xCenter + targets[1].xCenter)/2;
                int targetCenterY = (targets[0].yCenter + targets[1].yCenter)/2;
                int targetWidth = Math.abs(targets[0].xCenter - targets[1].xCenter) +
                                  (targets[0].width + targets[1].width)/2;
                int targetHeight = Math.max(targets[0].yCenter + targets[0].height/2,
                                            targets[1].yCenter + targets[1].height/2) -
                                   Math.min(targets[0].yCenter - targets[0].height/2,
                                            targets[1].yCenter - targets[1].height/2);
                int temp;

                switch (orientation)
                {
                    case CLOCKWISE_PORTRAIT:
                        temp = RobotInfo.PIXYCAM_WIDTH - targetCenterX;
                        targetCenterX = targetCenterY;
                        targetCenterY = temp;
                        temp = targetWidth;
                        targetWidth = targetHeight;
                        targetHeight = temp;
                        break;

                    case ANTICLOCKWISE_PORTRAIT:
                        temp = targetCenterX;
                        targetCenterX = RobotInfo.PIXYCAM_HEIGHT - targetCenterY;
                        targetCenterY = temp;
                        temp = targetWidth;
                        targetWidth = targetHeight;
                        targetHeight = temp;
                        break;

                    case UPSIDEDOWN_LANDSCAPE:
                        targetCenterX = RobotInfo.PIXYCAM_WIDTH - targetCenterX;
                        targetCenterY = RobotInfo.PIXYCAM_HEIGHT - targetCenterY;
                        break;

                    case NORMAL_LANDSCAPE:
                        break;
                }

                targetRect = new Rect(targetCenterX - targetWidth/2, targetCenterY - targetHeight/2,
                                      targetWidth, targetHeight);

                if (debugEnabled)
                {
                    dbgTrace.traceInfo("PixyVision", "TargetRect: x=%d, y=%d, w=%d, h=%d",
                        targetRect.x, targetRect.y, targetRect.width, targetRect.height);
                }
            }
        }

        return targetRect;
    }   //getTargetRect

    public TargetInfo getTargetInfo()
    {
        TargetInfo targetInfo = null;
        Rect targetRect = getTargetRect();

        if (targetRect != null)
        {
            //
            // Physical target width:           W = 10 inches.
            // Physical target distance 1:      D1 = 20 inches.
            // Target pixel width at 20 inches: w1 = 115
            // Physical target distance 2:      D2 = 24 inches
            // Target pixel width at 24 inches: w2 = 96
            // Camera lens focal length:        f
            //    W/D1 = w1/f and W/D2 = w2/f
            // => f = w1*D1/W and f = w2*D2/W
            // => w1*D1/W = w2*D2/W
            // => w1*D1 = w2*D2 = PIXY_DISTANCE_SCALE = 2300
            //
            // Screen center X:                 Xs = 320/2 = 160
            // Target center X:                 Xt
            // Heading error:                   e = Xt - Xs
            // Turn angle:                      a
            //    tan(a) = e/f
            // => a = atan(e/f) and f = w1*D1/W
            // => a = atan((e*W)/(w1*D1))
            //
            double targetCenterX = targetRect.x + targetRect.width/2.0;
            double targetDistance = PIXY_DISTANCE_SCALE/targetRect.width; 
            double targetAngle = Math.toDegrees(
                Math.atan((targetCenterX - RobotInfo.PIXYCAM_WIDTH/2.0)*TARGET_WIDTH_INCHES/
                          (targetDistance*targetRect.width)));
            targetInfo = new TargetInfo(targetRect, targetDistance, targetAngle);
        }

        return targetInfo;
    }   //getTargetInfo

}   // class PixyVision
