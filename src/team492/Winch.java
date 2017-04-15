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
import trclib.TrcDbgTrace;

public class Winch
{
    private static final String module = "Winch";
    private TrcDbgTrace tracer = TrcDbgTrace.getGlobalTracer();

    private FrcCANTalon motor1;
    private FrcCANTalon motor2;
    private boolean manualOverride = false;
    private boolean offGround = false;

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
        return !motor1.isUpperLimitSwitchActive();
    }

    public boolean isLowerLimitSwitchActive()
    {
        return !motor1.isLowerLimitSwitchActive();
    }

    public double getPosition()
    {
        return motor1.getPosition()*RobotInfo.WINCH_POSITION_SCALE; 
    }

    public void setPower(double power)
    {
        double motorPower = power;

        if (!offGround && getCurrent() >= RobotInfo.WINCH_MOTOR_CURRENT_THRESHOLD)
        {
            offGround = true;
            motor1.resetPosition();
        }

        if (!manualOverride)
        {
            if (touchingPlate())
            {
                motorPower = 0.0;
            }
            else if (offGround && getPosition() >= RobotInfo.WINCH_HEIGHT_THRESHOLD)
            {
                motorPower = power*RobotInfo.WINCH_MOTOR_POWER_SCALE;
            }
        }

        motor1.setPower(motorPower);
        motor2.setPower(motorPower);
    }

    private boolean touchingPlate()
    {
        return isUpperLimitSwitchActive() || isLowerLimitSwitchActive();
    }

    private double getCurrent()
    {
        double current1 = motor1.motor.getOutputCurrent();
        double current2 = motor2.motor.getOutputCurrent();
        tracer.traceInfo(module, "motor1Current=%.1f, motor2Current=%.1f", current1, current2);
        return Math.abs(current1) + Math.abs(current2);
    }

}   //class Winch
