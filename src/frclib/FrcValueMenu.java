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

package frclib;

import hallib.HalDashboard;
import trclib.TrcDbgTrace;

/**
 * This class implements a value menu where a default value is displayed. The user can press the UP and DOWN button
 * to increase or decrease the value and press the ENTER button to select the value. The user can also press the
 * BACK button to cancel the menu and go back to the parent menu.
 */
public class FrcValueMenu
{
    private static final String moduleName = "FrcValueMenu";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    private final String menuTitle;
    private double defaultValue;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param menuTitle specifies the title of the menu. The title will be displayed as the first line in the menu.
     * @param defaultValue specifies the default value.
     */
    public FrcValueMenu(String menuTitle, double defaultValue)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + menuTitle, tracingEnabled, traceLevel, msgLevel);
        }

        if (menuTitle == null)
        {
            throw new NullPointerException("menuTitle cannot be null.");
        }

        this.menuTitle = menuTitle;
        this.defaultValue = defaultValue;
        HalDashboard.putNumber(menuTitle, defaultValue);
    }   //FrcValueMenu

    /**
     * This method returns the title text of this menu.
     *
     * @return title text.
     */
    public String getTitle()
    {
        final String funcName = "getTitle";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", menuTitle);
        }

        return menuTitle;
    }   //getTitle

    /**
     * This method returns the current value of the value menu. Every value menu has a current value even if the menu
     * hasn't been displayed and the user hasn't changed the value. In that case, the current value is the default
     * value.
     *
     * @return current value of the value menu.
     */
    public double getCurrentValue()
    {
        final String funcName = "get`CurrentValue";
        double value = HalDashboard.getNumber(menuTitle, defaultValue);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", value);
        }

        return value;
    }   //getCurrentValue

}   //class FrcValueMenu
