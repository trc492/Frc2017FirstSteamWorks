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
    private static final double Range = 1.65;   //MTS: by convention, use ALL-CAP. May want to move it to RobotInfo. May want to call it PIXY_MAX_VOLTAGE

    public AnalogInput pixyCamera = null;   // MTS: why public?
    public DigitalInput objectDetected = null;  // MTS: why public?

    public PixyVision(final String instanceName)    // MTS: If you are not using instanceName, don't declare it!
    {
        pixyCamera = new AnalogInput(1);        // MTS: Create a constant in RobotInfo
        objectDetected = new DigitalInput(9);   // MTS: Create a constant in RobotInfo
    }

    public boolean isTargetDetected()
    {
        return objectDetected.get();
    }

    public double getTargetPosition()
    {
        double voltage = pixyCamera.getVoltage();
        return (voltage - Range) / Range;   //MTS: Wrong equation!
    }
}
