/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
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

import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import frclib.FrcCANTalon;

public class Shooter
{
    private FrcCANTalon motor;
    double setPoint = 0.0;

    public Shooter()
    {
        motor = new FrcCANTalon("Shooter", RobotInfo.CANID_SHOOTER);
        motor.setInverted(false);
        motor.motor.enableLimitSwitch(false, false);
        motor.setPositionSensorInverted(true);
        motor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        motor.motor.configEncoderCodesPerRev(RobotInfo.SHOOTER_COUNT_PER_REV);
        motor.motor.changeControlMode(TalonControlMode.Speed);
    }

    public void setPID(double p, double i, double d, double f, int izone, double closeLoopRampRate, int profile)
    {
        motor.motor.setPID(p, i, d, f, izone, closeLoopRampRate, profile);
    }

    public double getSetPoint()
    {
        return setPoint;
    }

    public double getSpeed()
    {
        return motor.motor.getSpeed();
    }

    public void setSpeed(double rpm)
    {
        //
        // counts_per_second = rpm*counts_per_rev/60
        // counts_per_10msec = rpm*counts_per_rev*100/60
        //
        setPoint = rpm*100.0/60.0;
        motor.motor.set(setPoint);
    }

    public double getPower()
    {
        return motor.getPower();
    }

    public void setPower(double power)
    {
        motor.setPower(power);
    }

    public double getCurrent()
    {
        return motor.motor.getOutputCurrent();
    }

}   //class Shooter
