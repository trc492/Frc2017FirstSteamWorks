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
import hallib.HalDashboard;
import trclib.TrcDbgTrace;
import trclib.TrcUtil;

public class Winch
{
    private static final String module = "Winch";
    private TrcDbgTrace tracer = TrcDbgTrace.getGlobalTracer();

    private FrcCANTalon motor1;
    private FrcCANTalon motor2;
    private double motorPower = 0.0;
    private boolean manualOverride = false;
    private boolean motorStarted = false;
    private boolean offGround = false;
    private double settlingTime = 0.0;
    private double maxCurrent = 0.0;

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

    public boolean isUpperLimitSwitchActive()
    {
        return motor1.isUpperLimitSwitchActive();
    }

    public boolean isLowerLimitSwitchActive()
    {
        return motor1.isLowerLimitSwitchActive();
    }

    public double getPosition()
    {
        return motor1.getPosition()*RobotInfo.WINCH_POSITION_SCALE; 
    }

    public void setPower(double power)
    {
        double currTime = TrcUtil.getCurrentTime();

        power = Math.abs(power);
        if (power == 0.0)
        {
            motorStarted = false;
            HalDashboard.getInstance().displayPrintf(11, "");
            HalDashboard.getInstance().displayPrintf(12, "");
        }
        else if (!motorStarted)
        {
            motorStarted = true;
            settlingTime = currTime + 0.5;
        }

        if (!offGround && currTime >= settlingTime && getCurrent() >= RobotInfo.WINCH_MOTOR_CURRENT_THRESHOLD)
        {
            offGround = true;
            motor1.resetPosition();
            HalDashboard.getInstance().displayPrintf(11, "reset!!!");
        }

        if (!manualOverride)
        {
            if (touchingPlate())
            {
                power = 0.0;
            }
            else if (offGround && getPosition() >= RobotInfo.WINCH_HEIGHT_THRESHOLD)
            {
                power *= RobotInfo.WINCH_MOTOR_POWER_SCALE;
                HalDashboard.getInstance().displayPrintf(12, "SLOW!!!");
            }
        }

        motorPower = power;
        motor1.setPower(motorPower);
        motor2.setPower(motorPower);
    }

    private boolean touchingPlate()
    {
        return isUpperLimitSwitchActive() || isLowerLimitSwitchActive();
    }

    public double getPower()
    {
        return motorPower;
    }

    public double getCurrent()
    {
        double current1 = motor1.motor.getOutputCurrent();
        double current2 = motor2.motor.getOutputCurrent();
        double totalCurrent = Math.abs(current1) + Math.abs(current2);

        if (totalCurrent > maxCurrent) maxCurrent = totalCurrent;

        return totalCurrent;
    }

    public double getMaxCurrent()
    {
        return maxCurrent;
    }

}   //class Winch
