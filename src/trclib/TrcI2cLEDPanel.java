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

package trclib;

/**
 * This class implements a platform independent I2C LED panel. It provides methods to display text data on the LED
 * panel.
 */
public abstract class TrcI2cLEDPanel extends TrcSerialBusDevice
{
    private static final int I2C_BUFF_LEN = 32;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcI2cLEDPanel(final String instanceName)
    {
        super(instanceName);
    }   //TrcI2cLEDPanel

    /**
     * This method sets the specified line in the LED panel with all the text info for displaying text on the panel.
     * Note that the (x, y) coordinates is rotation sensitive. If rotation is 0, the text orientation is normal
     * horizontal and (0, 0) corresponds to the upper left corner of the physical panel. If rotation is 2, the
     * text orientation is inverted horizontal and (0, 0) corresponds to the lower right corner of the physica
     * panel.
     *
     * @param index specifies the line index of the array.
     * @param x specifies the x coordinate of the upper left corner of the text rectangle.
     * @param y specifies the y coordinate of the upper left corner of the text rectangle.
     * @param fontColor specifies the font color for displaying the text.
     * @param orientaton specifies the text orientation (0: normal horizontal, 1: clockwise vertical, 
     *                   2: inverted horizontal, 3: anti-clockwise vertical).
     * @param fontSize specifies the size of the font (1: 6x8, 2: 12x16, 3: 18x24, 4: 24x32).
     * @param scrollInc specifies the scroll increment (0: no scroll, 1: scroll to the right, -1: scroll to the left).
     * @param text specifies the text string to be displayed.
     */
    public void setTextLine(
        int index, int x, int y, int fontColor, int orientation, int fontSize, int scrollInc, String text)
    {
        sendRequest("setTextLine " + index + " " + x + " " + y + " " + fontColor + " " + orientation + " " +
                    fontSize + " " + scrollInc + " " + text);
    }   //setTextLine

    /**
     * This method clears the specified line in the lines array.
     *
     * @param index specifies the line index of the array.
     */
    public void clearTextLine(int index)
    {
        sendRequest("clearTextLine " + index);
    } //clearTextLine

    /**
     * This method clears all text lines in the lines array.
     */
    public void clearAllTextLines()
    {
        sendRequest("clearAllTextLines");
    } //clearAllTextLines

    /**
     * This method sets the Arduino loop delay. This effectively controls how fast the text will scroll.
     *
     * @param delay specifies the delay in msec.
     */
    public void setDelay(int delay)
    {
        sendRequest("setDelay " + delay);
    }   //setDelay

    /**
     * This method converts the specified RGB values into a 16-bit color value in 565 format (5-bit R, 6-bit G and
     * 5-bit B: RRRRRGGGGGGBBBBB).
     *
     * @param red specifies the red value.
     * @param green specifies the green value.
     * @param blue specifies the blue value.
     * @return 16-bit color value in 565 format.
     */
    public int color(int red, int green, int blue)
    {
        return ((red & 0xff) << 11) | ((green & 0xff) << 5) | (blue & 0xff);
    }   //color

    /**
     * This method sends the request string to the I2C device. If the request string is longer than 32 bytes,
     * it will break down the request string into multiple I2C requests so that they can be reassembled on the
     * device side.
     *
     * @param request specifies the request string to be sent to the I2C device.
     */
    private void sendRequest(String request)
    {
        request += "~";
        int requestLen = request.length();
        TrcDbgTrace.getGlobalTracer().traceInfo("sendRequest", "Request: <%s> = %d", request, request.length());

        for (int i = 0; i < requestLen; )
        {
            int len = Math.min(requestLen - i, I2C_BUFF_LEN);
            if (len > 0)
            {
                String s = request.substring(i, i + len);
                byte[] data = s.getBytes();
                int n = writeData(-1, data, data.length);

                if (n == 0)
                {
                    break;
                }
                i += len;
            }
        }
    }   //sendRequest

}   //class TrcI2cLEDPanel
