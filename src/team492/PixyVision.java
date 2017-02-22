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
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import frclib.FrcPixyCam;
import frclib.FrcPixyCam.ObjectBlock;
import frclib.FrcRobotBase;
import trclib.TrcDbgTrace;

public class PixyVision
{
    private FrcPixyCam pixyCamera = null;
    private Relay ringLightPower = null;
    private TrcDbgTrace tracer = FrcRobotBase.getGlobalTracer();

    public PixyVision()
    {
        pixyCamera = new FrcPixyCam("FrontPixy", I2C.Port.kMXP, RobotInfo.PIXYCAM_FRONT_I2C_ADDRESS);
        ringLightPower = new Relay(RobotInfo.RELAY_RINGLIGHT_POWER);
        ringLightPower.setDirection(Direction.kForward);
    }

    public void setRingLightOn(boolean on)
    {
        ringLightPower.set(on? Value.kOn: Value.kOff);
    }

    /**
     * This method returns an array of rectangles of last detected objects.
     *
     * @return array of rectangle of last detected objects.
     */
    public Rect[] getObjectRects()
    {
        Rect[] objectRects = null;
        ObjectBlock[] detectedObjects = pixyCamera.getDetectedObjects();

        if (detectedObjects != null)
        {
            for (int i = 0; i < detectedObjects.length; i++)
            {
                tracer.traceInfo("PixyVision", "[%d] %s", i, detectedObjects[i].toString());
            }
        }

        return objectRects;
    }   //getObjectRects

}   // class PixyVision
