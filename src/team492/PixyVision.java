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

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;

public class PixyVision
{
    private AnalogInput pixyCamera = null;
    private DigitalInput objectDetected = null;
    private Relay ringLightPower = null;

    public PixyVision()
    {
        pixyCamera = new AnalogInput(RobotInfo.AIN_PIXYCAM_OBJECT_POS);
        objectDetected = new DigitalInput(RobotInfo.DIN_PIXYCAM_OBJECTS_DETECTED);
        ringLightPower = new Relay(RobotInfo.RELAY_RINGLIGHT_POWER);
        ringLightPower.setDirection(Direction.kForward);
    }

    public void setRingLightOn(boolean on)
    {
        ringLightPower.set(on? Value.kOn: Value.kOff);
    }

    public boolean isTargetDetected()
    {
        return objectDetected.get();
    }

    public double getTargetPosition()
    {
        double voltage = pixyCamera.getVoltage();
        return (voltage - RobotInfo.PIXYCAM_MID_VOLT) / RobotInfo.PIXYCAM_MID_VOLT;
    }
//  /**
//  * This method returns an array of rectangles of last detected objects.
//  *
//  * @return array of rectangle of last detected objects.
//  */
// public Rect[] getObjectRects()
// {
//     final String funcName = "getObjectRects";
//     Rect[] objectRects = null;
//
//     if (debugEnabled)
//     {
//         dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
//         dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
//     }
//
//     synchronized (objectLock)
//     {
//         objectRects = detectedObjectRects;
//         detectedObjectRects = null;
//     }
//
//     return objectRects;
// }   //getObjectRects

}
