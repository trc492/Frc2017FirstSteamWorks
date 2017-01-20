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

package frclib;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import trclib.TrcDbgTrace;
import trclib.TrcRobot;
import trclib.TrcTaskMgr;

public class FrcJoystick extends Joystick implements TrcTaskMgr.Task
{
    private static final String moduleName = "FrcJoystick";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    //
    // Logitech Joystick:
    // UsagePage=0x01, Usage-0x04
    //
    public static final int LOGITECH_TRIGGER    = (1 << 0);
    public static final int LOGITECH_BUTTON2    = (1 << 1);
    public static final int LOGITECH_BUTTON3    = (1 << 2);
    public static final int LOGITECH_BUTTON4    = (1 << 3);
    public static final int LOGITECH_BUTTON5    = (1 << 4);
    public static final int LOGITECH_BUTTON6    = (1 << 5);
    public static final int LOGITECH_BUTTON7    = (1 << 6);
    public static final int LOGITECH_BUTTON8    = (1 << 7);
    public static final int LOGITECH_BUTTON9    = (1 << 8);
    public static final int LOGITECH_BUTTON10   = (1 << 9);
    public static final int LOGITECH_BUTTON11   = (1 << 10);
    public static final int LOGITECH_BUTTON12   = (1 << 11);
    //
    // Logitech DualAction Game Controller:
    // UsagePage=0x01, Usage-0x04
    //
    public static final int DUALACTION_BUTTONX  = (1 << 0);
    public static final int DUALACTION_BUTTONA  = (1 << 1);
    public static final int DUALACTION_BUTTONB  = (1 << 2);
    public static final int DUALACTION_BUTTONY  = (1 << 3);
    public static final int DUALACTION_LB       = (1 << 4);
    public static final int DUALACTION_RB       = (1 << 5);
    public static final int DUALACTION_LT       = (1 << 6);
    public static final int DUALACTION_RT       = (1 << 7);
    public static final int DUALACTION_BACK     = (1 << 8);
    public static final int DUALACTION_START    = (1 << 9);
    public static final int DUALACTION_LTOP     = (1 << 10);
    public static final int DUALACTION_RTOP     = (1 << 11);
    //
    // Microsoft SideWinder Joystick:
    // UsagePage=0x01, Usage-0x04
    //
    public static final int SIDEWINDER_TRIGGER  = (1 << 0);
    public static final int SIDEWINDER_BUTTON2  = (1 << 1);
    public static final int SIDEWINDER_BUTTON3  = (1 << 2);
    public static final int SIDEWINDER_BUTTON4  = (1 << 3);
    public static final int SIDEWINDER_BUTTON5  = (1 << 4);
    public static final int SIDEWINDER_BUTTON6  = (1 << 5);
    public static final int SIDEWINDER_BUTTON7  = (1 << 6);
    public static final int SIDEWINDER_BUTTON8  = (1 << 7);
    public static final int SIDEWINDER_BUTTON9  = (1 << 8);
    //
    // Microsoft XBox Game Controller:
    // UsagePage=0x01, Usage-0x05
    //
    public static final int XBOX_BUTTONA        = (1 << 0);
    public static final int XBOX_BUTTONB        = (1 << 1);
    public static final int XBOX_BUTTONX        = (1 << 2);
    public static final int XBOX_BUTTONY        = (1 << 3);
    public static final int XBOX_LB             = (1 << 4);
    public static final int XBOX_RB             = (1 << 5);
    public static final int XBOX_BACK           = (1 << 6);
    public static final int XBOX_START          = (1 << 7);

    private double deadbandThreshold = 0.15;

    public interface ButtonHandler
    {
        public void joystickButtonEvent(FrcJoystick joystick, int button, boolean pressed);
    }   //interface ButonHandler

    private int port;
    private ButtonHandler buttonHandler;
    private DriverStation ds;
    private int prevButtons;
    private int ySign;

    public FrcJoystick(
            final String instanceName,
            final int port,
            ButtonHandler buttonHandler)
    {
        super(port);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName + "." + instanceName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        this.port = port;
        this.buttonHandler = buttonHandler;
        ds = DriverStation.getInstance();
        prevButtons = ds.getStickButtons(port);
        ySign = 1;
        TrcTaskMgr.getInstance().registerTask(
                instanceName,
                this,
                TrcTaskMgr.TaskType.PREPERIODIC_TASK);
    }   //FrcJoystick

    public FrcJoystick(
            final String instanceName,
            final int port,
            ButtonHandler buttonHandler,
            final double deadbandThreshold)
    {
        this(instanceName, port, buttonHandler);
        this.deadbandThreshold = deadbandThreshold;
    }   //FrcJoystick

    public void setYInverted(boolean inverted)
    {
        final String funcName = "setYInverted";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "inverted=%s",
                    Boolean.toString(inverted));
        }

        ySign = inverted? -1: 1;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //set YInverted

    public double getXWithDeadband(boolean squared, double deadbandThreshold)
    {
        final String funcName = "getXWithDeadband";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "squared=%s,dbThreshold=%f",
                    Boolean.toString(squared), deadbandThreshold);
        }

        double value = adjustValueWithDeadband(
                getX(Hand.kRight),
                squared,
                deadbandThreshold);

        if (debugEnabled)
        {
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", value);
        }
        return value;
    }   //getXWithDeadband

    public double getXWithDeadband(boolean squared)
    {
        return getXWithDeadband(squared, deadbandThreshold);
    }   //getXWithDeadband

    public double getYWithDeadband(boolean squared, double deadbandThreshold)
    {
        final String funcName = "getYWithDeadband";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "squared=%s,dbThreshold=%f",
                    Boolean.toString(squared), deadbandThreshold);
        }

        double value = adjustValueWithDeadband(
                ySign*getY(Hand.kRight),
                squared,
                deadbandThreshold);

        if (debugEnabled)
        {
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", value);
        }
        return value;
    }   //getYWithDeadband

    public double getYWithDeadband(boolean squared)
    {
        return getYWithDeadband(squared, deadbandThreshold);
    }   //getYWithDeadband

    public double getZWithDeadband(boolean squared, double deadbandThreshold)
    {
        final String funcName = "getZWithDeadband";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "squared=%s,dbThreshold=%f",
                    Boolean.toString(squared), deadbandThreshold);
        }

        double value = adjustValueWithDeadband(
                getZ(Hand.kRight),
                squared,
                deadbandThreshold);

        if (debugEnabled)
        {
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", value);
        }
        return value;
    }   //getZWithDeadband

    public double getZWithDeadband(boolean squared)
    {
        return getZWithDeadband(squared, deadbandThreshold);
    }   //getZWithDeadband

    public double getTwistWithDeadband(
            boolean squared,
            double deadbandThreshold)
    {
        final String funcName = "getTwistWithDeadband";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "squared=%s,dbThreshold=%f",
                    Boolean.toString(squared), deadbandThreshold);
        }

        double value = adjustValueWithDeadband(
                getTwist(),
                squared,
                deadbandThreshold);

        if (debugEnabled)
        {
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", value);
        }
        return value;
    }   //getTwistWithDeadband

    public double getTwistWithDeadband(boolean squared)
    {
        return getTwistWithDeadband(squared, deadbandThreshold);
    }   //getTwistWithDeadband

    public double getThrottleWithDeadband(
            boolean squared,
            double deadbandThreshold)
    {
        final String funcName = "getThrottleWithDeadband";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "squared=%s,dbThreshold=%f",
                    Boolean.toString(squared), deadbandThreshold);
        }

        double value = adjustValueWithDeadband(
                getThrottle(),
                squared,
                deadbandThreshold);

        if (debugEnabled)
        {
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", value);
        }
        return value;
    }   //getThrottleWithDeadband

    public double getThrottleWithDeadband(boolean squared)
    {
        return getThrottleWithDeadband(squared, deadbandThreshold);
    }   //getThrottleWithDeadband

    public double getMagnitudeWithDeadband(
            boolean squared,
            double deadbandThreshold)
    {
        final String funcName = "getMagnitudeWithDeadband";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "squared=%s,dbThreshold=%f",
                    Boolean.toString(squared), deadbandThreshold);
        }

        double value = adjustValueWithDeadband(
                getMagnitude(),
                squared,
                deadbandThreshold);

        if (debugEnabled)
        {
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", value);
        }
        return value;
    }   //getMagnitudeWithDeadband

    public double getMagnitudeWithDeadband(boolean squared)
    {
        return getMagnitudeWithDeadband(squared, deadbandThreshold);
    }   //getMagnitudeWithDeadband

    public double getDirectionRadiansWithDeadband(
            boolean squared,
            double deadbandThreshold)
    {
        final String funcName = "getDirectionRadiansWithDeadband";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "squared=%s,dbThreshold=%f",
                    Boolean.toString(squared), deadbandThreshold);
        }

        double value = adjustValueWithDeadband(
                getDirectionRadians(),
                squared,
                deadbandThreshold);

        if (debugEnabled)
        {
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", value);
        }
        return value;
    }   //getDirectionRadiansWithDeadband

    public double getDirectionRadiansWithDeadband(boolean squared)
    {
        return getDirectionRadiansWithDeadband(squared, deadbandThreshold);
    }   //getDirectionRadiansWithDeadband

    public double getDirectionDegreesWithDeadband(
            boolean squared,
            double deadbandThreshold)
    {
        final String funcName = "getDirectionDegreesWithDeadband";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "squared=%s,dbThreshold=%f",
                    Boolean.toString(squared), deadbandThreshold);
        }

        double value = adjustValueWithDeadband(
                getDirectionDegrees(),
                squared,
                deadbandThreshold);

        if (debugEnabled)
        {
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", value);
        }
        return value;
    }   //getDirectionDegreesWithDeadband

    public double getDirectionDegreesWithDeadband(boolean squared)
    {
        return getDirectionDegreesWithDeadband(squared, deadbandThreshold);
    }   //getDirectionDegreesWithDeadband

    //
    // Implements TrcTaskMgr.Task
    //
    public void startTask(TrcRobot.RunMode runMode)
    {
    }   //startTask

    public void stopTask(TrcRobot.RunMode runMode)
    {
    }   //stopTask

    public void prePeriodicTask(TrcRobot.RunMode runMode)
    {
        final String funcName = "prePeriodic";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.TASK,
                    "mode=%s", runMode.toString());
        }

        int currButtons = ds.getStickButtons(port);
        if (buttonHandler != null && runMode != TrcRobot.RunMode.DISABLED_MODE)
        {
            int changedButtons = prevButtons^currButtons;
            int buttonMask;
            while (changedButtons != 0)
            {
            	//
                // buttonMask contains the least significant set bit.
                //
                buttonMask = changedButtons & ~(changedButtons^-changedButtons);
                if ((currButtons & buttonMask) != 0)
                {
                    //
                    // Button is pressed.
                    //
                    if (debugEnabled)
                    {
                        dbgTrace.traceInfo(
                                funcName,
                                "Button %x pressed",
                                buttonMask);
                    }
                    buttonHandler.joystickButtonEvent(
                            this, buttonMask, true);
                }
                else
                {
                    //
                    // Button is released.
                    //
                    if (debugEnabled)
                    {
                        dbgTrace.traceInfo(
                                funcName,
                                "Button %x released",
                                buttonMask);
                    }
                    buttonHandler.joystickButtonEvent(
                            this, buttonMask, false);
                }
                //
                // Clear the least significant set bit.
                //
                changedButtons &= ~buttonMask;
            }
        }
        prevButtons = currButtons;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //prePeriodicTask

    public void postPeriodicTask(TrcRobot.RunMode runMode)
    {
    }   //postPeriodicTask

    public void preContinuousTask(TrcRobot.RunMode runMode)
    {
    }   //preContinuousTask

    public void postContinuousTask(TrcRobot.RunMode runMode)
    {
    }   //postContinuousTask

    private double adjustValueWithDeadband(
            double value,
            boolean squared,
            double deadbandThreshold)
    {
        value = (Math.abs(value) >= deadbandThreshold)? value: 0.0;
        if (squared)
        {
            int dir = (value >= 0.0)? 1: -1;
            value = dir*value*value;
        }
        return value;
    }   //adjustValueWithDeadband

}   //class FrcJoystick
