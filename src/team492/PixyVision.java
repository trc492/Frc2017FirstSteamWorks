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

public class PixyVision
{
    private AnalogInput pixyCamera = null;
    private DigitalInput objectDetected = null;

    public PixyVision()
    {
        pixyCamera = new AnalogInput(RobotInfo.AIN_PIXYCAM_OBJECT_POS);
        objectDetected = new DigitalInput(RobotInfo.DIN_PIXYCAM_OBJECTS_DETECTED);
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
}
