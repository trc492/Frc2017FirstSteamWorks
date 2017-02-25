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

//import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import frclib.FrcPixyCam;
import frclib.FrcPixyCam.ObjectBlock;
import trclib.TrcDbgTrace;

public class PixyVision
{
    private static final String moduleName = "PixyVision";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    public enum Orientation
    {
        NORMAL_LANDSCAPE,
        CLOCKWISE_PORTRAIT,
        ANTICLOCKWISE_PORTRAIT,
        UPSIDEDOWN_LANDSCAPE
    }   //enum Orientation

    private int signature;
    private Orientation orientation;
    private FrcPixyCam pixyCamera = null;
    private Relay ringLightPower = null;

    public PixyVision(int signature, int brightness, Orientation orientation)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        this.signature = signature;
        this.orientation = orientation;
//        pixyCamera = new FrcPixyCam("FrontPixy", I2C.Port.kOnboard, RobotInfo.PIXYCAM_FRONT_I2C_ADDRESS);
        pixyCamera = new FrcPixyCam("RearPixy", SerialPort.Port.kMXP, 19200);
        pixyCamera.setBrightness((byte)brightness);
        ringLightPower = new Relay(RobotInfo.RELAY_RINGLIGHT_POWER);
        ringLightPower.setDirection(Direction.kForward);
    }

    public void setEnabled(boolean enabled)
    {
        pixyCamera.setEnabled(enabled);
    }   //setEnabled

    public boolean isEnabled()
    {
        return pixyCamera.isEnabled();
    }   //isEnabled

    public void setRingLightOn(boolean on)
    {
        ringLightPower.set(on? Value.kOn: Value.kOff);
    }

    /**
     * This method returns the rectangle of the last detected target.
     *
     * @return rectangle of last detected target.
     */
    public Rect getTargetRect()
    {
        Rect targetRect = null;
        ObjectBlock[] detectedObjects = pixyCamera.getDetectedObjects();

        if (debugEnabled)
        {
            dbgTrace.traceInfo(moduleName, "%d object(s) found", detectedObjects != null? detectedObjects.length: 0);
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

}   // class PixyVision
