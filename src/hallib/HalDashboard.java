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

package hallib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import trclib.TrcDbgTrace;

/**
 * This class is a wrapper for the Telemetry class. In addition to providing
 * a way to send named data to the Driver Station to be displayed, it also
 * simulates an LCD display similar to the NXT Mindstorms. The Mindstorms
 * has only 8 lines but this dashboard can support as many lines as the
 * Driver Station can support. By default, we set the number of lines to 16.
 * By changing a constant here, you can have as many lines as you want. This
 * dashboard display is very useful for displaying debug information. In
 * particular, the TrcMenu class uses the dashboard to display a choice menu
 * and interact with the user for choosing autonomous strategies and options.
 */
public class HalDashboard extends SmartDashboard
{
    private static final String moduleName = "HalDashboard";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    public static final int MAX_NUM_TEXTLINES = 16;

    private static final String displayKeyFormat = "%02d";
    private static HalDashboard instance = null;
    private static String[] display = new String[MAX_NUM_TEXTLINES];

    /**
     * Constructor: Creates an instance of the object.
     * There should only be one global instance of this object.
     * Typically, only the FtcOpMode object should construct an
     * instance of this object via getInstance(telemetry) and
     * nobody else.
     */
    public HalDashboard()
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        instance = this;
        clearDisplay();
    }   //HalDashboard

    /**
     * This static method allows any class to get an instance of
     * the dashboard so that it can display information on its
     * display.
     *
     * @return global instance of the dashboard object.
     */
    public static HalDashboard getInstance()
    {
        return instance;
    }   //getInstance

    /**
     * This method displays a formatted message to the display on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param format specifies the format string.
     * @param args specifies variable number of substitution arguments.
     */
    public void displayPrintf(int lineNum, String format, Object... args)
    {
        if (lineNum >= 0 && lineNum < display.length)
        {
            display[lineNum] = String.format(format, args);
            SmartDashboard.putString(
                    String.format(displayKeyFormat, lineNum), display[lineNum]);
        }
    }   //displayPrintf

    /**
     * This method clears all the display lines.
     */
    public void clearDisplay()
    {
        final String funcName = "clearDisplay";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        for (int i = 0; i < display.length; i++)
        {
            display[i] = "";
        }
        refreshDisplay();
    }   //clearDisplay

    /**
     * This method refresh the display lines to the Driver Station.
     */
    public void refreshDisplay()
    {
        final String funcName = "refreshDisplay";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        for (int i = 0; i < display.length; i++)
        {
            SmartDashboard.putString(
                    String.format(displayKeyFormat, i), display[i]);
        }
    }   //refreshDisplay

}   //class HalDashboard
