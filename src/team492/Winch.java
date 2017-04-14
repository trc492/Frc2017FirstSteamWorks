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

import frclib.FrcCANTalon;

public class Winch
{
    private FrcCANTalon motor1;
    private FrcCANTalon motor2;
    private boolean manualOverride = false;

    public Winch()
    {
        motor1 = new FrcCANTalon("WinchMotor1", RobotInfo.CANID_WINCH1);
        motor2 = new FrcCANTalon("WinchMotor2", RobotInfo.CANID_WINCH2);
        motor1.setPositionSensorInverted(false);
    }

    public void setManualOverride(boolean override)
    {
        this.manualOverride = override;
    }

    public boolean isFwdLimitSwitchActive()
    {
        return !motor1.isFwdLimitSwitchClosed();
    }

    public boolean isRevLimitSwitchActive()
    {
        return !motor1.isRevLimitSwitchClosed();
    }

    public double getPosition()
    {
        return motor1.getPosition()*RobotInfo.WINCH_POSITION_SCALE; 
    }

    public void setPower(double power)
    {
        if (manualOverride || !isFwdLimitSwitchActive() && !isRevLimitSwitchActive())
        {
            motor1.setPower(power);
            motor2.setPower(power);
        }
        else
        {
            motor1.setPower(0.0);
            motor2.setPower(0.0);
        }
    }
  
}   //class Winch
