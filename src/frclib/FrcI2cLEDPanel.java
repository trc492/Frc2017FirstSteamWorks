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

import edu.wpi.first.wpilibj.I2C;
import trclib.TrcI2cLEDPanel;

/**
 * This class implements a platform dependent I2C LED panel device. It extends the platform independent counterpart
 * and provides platform dependent access to the I2C device.
 */
public class FrcI2cLEDPanel extends TrcI2cLEDPanel
{
    public static final I2C.Port DEF_I2C_PORT = I2C.Port.kOnboard;
    public static final int DEF_I2C_ADDRESS = 0x8;

    private FrcI2cDevice device = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the I2C port on the RoboRIO.
     * @param devAddress specifies the I2C address of the device.
     */
    public FrcI2cLEDPanel(final String instanceName, I2C.Port port, int devAddress)
    {
        super(instanceName);
        device = new FrcI2cDevice(instanceName, port, devAddress);
    }   //FrcI2cLEDPanel

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the I2C port on the RoboRIO.
     */
    public FrcI2cLEDPanel(final String instanceName, I2C.Port port)
    {
        this(instanceName, port, DEF_I2C_ADDRESS);
    }   //FrcI2cLEDPanel

    //
    // Implements TrcSerialBusDevice abstract methods.
    //

    /**
     * This method is called to read data from the device with the specified length.
     *
     * @param address specifies the I2C register address to read from if any.
     * @param length specifies the number of bytes to read.
     * @return a byte array containing the data read.
     */
    @Override
    public byte[] readData(int address, int length)
    {
        return device.readData(address, length);
    }   //readData

    /**
     * This method is called to write data to the device with the specified data buffer and length.
     *
     * @param address specifies the I2C register address to write to if any.
     * @param buffer specifies the buffer containing the data to be written to the device.
     * @param length specifies the number of bytes to write.
     * @return number of bytes written.
     */
    @Override
    public int writeData(int address, byte[] buffer, int length)
    {
        return device.writeData(address, buffer, length);
    }   //writeData

}   //class FrcI2cLEDPanel
