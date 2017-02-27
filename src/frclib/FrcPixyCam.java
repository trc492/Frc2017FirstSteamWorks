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

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import trclib.TrcDbgTrace;
import trclib.TrcDeviceQueue;
import trclib.TrcUtil;

/**
 * This class implements a platform dependent pixy camera that is either connected to an I2C bus or a Serial Port.
 * It provides access to the last detected objects reported by the pixy camera asynchronously.
 */
public class FrcPixyCam implements TrcDeviceQueue.CompletionHandler
{
    private static final String moduleName = "FrcPixyCam";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    public static final I2C.Port DEF_I2C_PORT = I2C.Port.kOnboard;
    public static final int DEF_I2C_ADDRESS = 0x54;

    public static final SerialPort.Port DEF_SERIAL_PORT = SerialPort.Port.kOnboard;
    public static final int DEF_BAUD_RATE = 19200;
    public static final int DEF_DATA_BITS = 8;
    public static final SerialPort.Parity DEF_PARITY = SerialPort.Parity.kNone;
    public static final SerialPort.StopBits DEF_STOP_BITS = SerialPort.StopBits.kOne;

    private static final byte PIXY_SYNC_HIGH                    = (byte)0xaa;
    private static final int PIXY_START_WORD                    = 0xaa55;
    private static final int PIXY_START_WORD_CC                 = 0xaa56;
    private static final int PIXY_START_WORDX                   = 0x55aa;

    private static final byte PIXY_CMD_SET_LED                  = (byte)0xfd;
    private static final byte PIXY_CMD_SET_BRIGHTNESS           = (byte)0xfe;
    private static final byte PIXY_CMD_SET_PAN_TILT             = (byte)0xff;

    /**
     * This class implements the pixy camera object block communication protocol. 
     */
    public class ObjectBlock
    {
        public int sync;
        public int checksum;
        public int signature;
        public int xCenter;
        public int yCenter;
        public int width;
        public int height;
        public int angle;

        public String toString()
        {
            return String.format(
                "sync=0x%04x, chksum=0x%04x, sig=%d, xCenter=%3d, yCenter=%3d, width=%3d, height=%3d, angle=%3d",
                sync, checksum, signature, xCenter, yCenter, width, height, angle);
        }
    }   //class ObjectBlock

    /**
     * This is used identify the request type.
     */
    private static enum RequestTag
    {
        SYNC,
        ALIGN,
        CHECKSUM,
        NORMAL_BLOCK,
        COLOR_CODE_BLOCK
    }   //enum RequestTag

    private FrcI2cDevice i2cPixy = null;
    private FrcSerialPort serialPixy = null;
    private ArrayList<ObjectBlock> objects = new ArrayList<>();
    private ObjectBlock[] detectedObjects = null;
    private ObjectBlock currBlock = null;
    private Object objectLock = new Object();

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the I2C port on the RoboRIO.
     * @param devAddress specifies the I2C address of the device.
     */
    public FrcPixyCam(final String instanceName, I2C.Port port, int devAddress)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        i2cPixy = new FrcI2cDevice(instanceName, port, devAddress);
    }   //FrcPixyCam

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the I2C port on the RoboRIO.
     */
    public FrcPixyCam(final String instanceName, I2C.Port port)
    {
        this(instanceName, port, DEF_I2C_ADDRESS);
    }   //FrcPixyCam

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the serial port on the RoboRIO.
     * @param baudRate specifies the baud rate.
     * @param dataBits specifies the number of data bits.
     * @param parity specifies the parity type.
     * @param stopBits specifies the number of stop bits.
     */
    public FrcPixyCam(
        final String instanceName, SerialPort.Port port, int baudRate, int dataBits, SerialPort.Parity parity,
        SerialPort.StopBits stopBits)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        serialPixy = new FrcSerialPort(instanceName, port, baudRate, dataBits, parity, stopBits);
    }   //FrcPixyCam

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the serial port on the RoboRIO.
     * @param baudRate specifies the baud rate.
     */
    public FrcPixyCam(final String instanceName, SerialPort.Port port, int baudRate)
    {
        this(instanceName, port, baudRate, DEF_DATA_BITS, DEF_PARITY, DEF_STOP_BITS);
    }   //FrcPixyCam

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the serial port on the RoboRIO.
     */
    public FrcPixyCam(final String instanceName, SerialPort.Port port)
    {
        this(instanceName, port, DEF_BAUD_RATE, DEF_DATA_BITS, DEF_PARITY, DEF_STOP_BITS);
    }   //FrcPixyCam

    /**
     * This method checks if the pixy camera is enabled.
     *
     * @return true if pixy camera is enabled, false otherwise.
     */
    public boolean isEnabled()
    {
        final String funcName = "isEnabled";
        boolean enabled = i2cPixy != null? i2cPixy.isTaskEnabled():
                          serialPixy != null? serialPixy.isTaskEnabled(): false;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(enabled));
        }

        return enabled;
    }   //isEnable

    /**
     * This method enables/disables the pixy camera.
     *
     * @param enabled specifies true to enable pixy camera, false to disable.
     */
    public void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";
        boolean wasEnabled = isEnabled();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enanbled=%s", Boolean.toString(enabled));
        }

        if (i2cPixy != null)
        {
            i2cPixy.setTaskEnabled(enabled);
        }
        else if (serialPixy != null)
        {
            serialPixy.setTaskEnabled(enabled);
        }

        if (!wasEnabled && enabled)
        {
            asyncReadData(RequestTag.SYNC, 2);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setEnabled

    /**
     * This method issues an asynchronous read of the specified number of bytes from the device.
     *
     * @param requestTag specifies the tag to identify the request. Can be null if none was provided.
     * @param length specifies the number of bytes to read.
     */
    public void asyncReadData(RequestTag requestTag, int length)
    {
        final String funcName = "asyncReadData";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "tag=%s,length=%d",
                requestTag != null? requestTag: "null", length);
        }

        if (i2cPixy != null)
        {
            i2cPixy.asyncRead(requestTag, length, null, this);
        }
        else if (serialPixy != null)
        {
            serialPixy.asyncRead(requestTag, length, null, this);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //asyncReadData

    /**
     * This method writes the data buffer to the device asynchronously.
     *
     * @param requestTag specifies the tag to identify the request. Can be null if none was provided.
     * @param data specifies the data buffer.
     */
    public void asyncWriteData(RequestTag requestTag, byte[] data)
    {
        final String funcName = "asyncWriteData";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "tag=%s,data=%s,length=%d",
                requestTag != null? requestTag: "null", Arrays.toString(data), data.length);
        }

        if (i2cPixy != null)
        {
            i2cPixy.asyncWrite(requestTag, data, data.length, null, null);
        }
        else if (serialPixy != null)
        {
            serialPixy.asyncWrite(requestTag, data, data.length, null, null);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //asyncWriteData

    /**
     * This method sets the LED to the specified color.
     *
     * @param red specifies the red value.
     * @param green specifies the green value.
     * @param blue specifies the blue value.
     */
    public void setLED(byte red, byte green, byte blue)
    {
        final String funcName = "setLED";
        byte[] data = new byte[5];

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "red=%d,green=%d,blue=%d", red, green, blue);
        }

        data[0] = 0x00;
        data[1] = PIXY_CMD_SET_LED;
        data[2] = red;
        data[3] = green;
        data[4] = blue;
        asyncWriteData(null, data);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setLED

    /**
     * This method sets the camera brightness.
     *
     * @param brightness specifies the brightness value.
     */
    public void setBrightness(byte brightness)
    {
        final String funcName = "setBrightness";
        byte[] data = new byte[3];

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "brightness=%d", brightness);
        }

        data[0] = 0x00;
        data[1] = PIXY_CMD_SET_BRIGHTNESS;
        data[2] = brightness;
        asyncWriteData(null, data);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setBrightness

    /**
     * This method sets the pan and tilt servo positions.
     * @param pan specifies the pan position between 0 and 1000.
     * @param tilt specifies the tilt position between 0 and 1000.
     */
    public void setPanTilt(int pan, int tilt)
    {
        final String funcName = "setPanTilt";
        byte[] data = new byte[6];

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "pan=%d,tilt=%d", pan, tilt);
        }

        if (pan < 0 || pan > 1000 || tilt < 0 || tilt > 1000)
        {
            throw new IllegalArgumentException("Invalid pan/tilt range.");
        }

        data[0] = 0x00;
        data[1] = PIXY_CMD_SET_PAN_TILT;
        data[2] = (byte)(pan & 0xff);
        data[3] = (byte)(pan >> 8);
        data[4] = (byte)(tilt & 0xff);
        data[5] = (byte)(tilt >> 8);
        asyncWriteData(null, data);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPanTilt

    /**
     * This method returns an array of detected object blocks.
     *
     * @return array of detected object blocks, can be null if no object detected.
     */
    public ObjectBlock[] getDetectedObjects()
    {
        final String funcName = "getDetectedObjects";
        ObjectBlock[] objectBlocks = null;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
}

        synchronized (objectLock)
        {
            objectBlocks = detectedObjects;
            detectedObjects = null;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        return objectBlocks;
    }   //getDetectedObjects

    /**
     * This method processes the data from the read completion handler.
     *
     * @param requestTag specifies the tag to identify the request. Can be null if none was provided.
     * @param data specifies the data read.
     * @param length specifies the number of bytes read.
     */
    private void processData(RequestTag requestTag, byte[] data, int length)
    {
        final String funcName = "processData";
        int word;

        if (debugEnabled)
        {
            dbgTrace.traceInfo(funcName, "tag=%s,data=%s,len=%d", requestTag, Arrays.toString(data), length);
        }

        switch (requestTag)
        {
            case SYNC:
                if (currBlock == null)
                {
                    currBlock = new ObjectBlock();
                }

                if (length != 2)
                {
                    throw new IllegalStateException(String.format("Unexpected data length %d in %s.",
                        length, requestTag));
                }
                else
                {
                    word = TrcUtil.bytesToInt(data[0], data[1]);
                    if (word == PIXY_START_WORD || word == PIXY_START_WORD_CC)
                    {
                        currBlock.sync = word;
                        asyncReadData(RequestTag.CHECKSUM, 2);
                    }
                    else if (word == PIXY_START_WORDX)
                    {
                        currBlock.sync = PIXY_START_WORD;
                        asyncReadData(RequestTag.ALIGN, 1);
                    }
                    else
                    {
                        asyncReadData(RequestTag.SYNC, 2);
                    }
                }
                break;

            case ALIGN:
                if (length != 1)
                {
                    throw new IllegalStateException(String.format("Unexpected data length %d in %s.",
                        length, requestTag));
                }
                else if (data[0] == PIXY_SYNC_HIGH)
                {
                    asyncReadData(RequestTag.CHECKSUM, 2);
                }
                else
                {
                    asyncReadData(RequestTag.SYNC, 2);
                }
                break;

            case CHECKSUM:
                if (length != 2)
                {
                    throw new IllegalStateException(String.format("Unexpected data length %d in %s.",
                        length, requestTag));
                }
                else
                {
                    word = TrcUtil.bytesToInt(data[0], data[1]);
                    if (word == PIXY_START_WORD || word == PIXY_START_WORD_CC)
                    {
                        currBlock.sync = word;
                        asyncReadData(RequestTag.CHECKSUM, 2);
                        //
                        // Detected end-of-frame, convert the array list of objects into detected object array.
                        //
                        if (objects.size() > 0)
                        {
                            synchronized (objectLock)
                            {
                                ObjectBlock[] array = new ObjectBlock[objects.size()];
                                detectedObjects = objects.toArray(array);
                                objects.clear();
                                if (debugEnabled)
                                {
                                    for (int i = 0; i < detectedObjects.length; i++)
                                    {
                                        dbgTrace.traceInfo(funcName, "[%02d] %s", i, detectedObjects[i].toString());
                                    }
                                }
                            }
                        }
                    }
                    else if (currBlock.sync == PIXY_START_WORD)
                    {
                        asyncReadData(RequestTag.NORMAL_BLOCK, 10);
                    }
                    else if (currBlock.sync == PIXY_START_WORD_CC)
                    {
                        asyncReadData(RequestTag.COLOR_CODE_BLOCK, 12);
                    }
                    else
                    {
                        throw new IllegalStateException(String.format("Unexpected sync word 0x%04x in %s.",
                            currBlock.sync, requestTag));
                    }
                }
                break;

            case NORMAL_BLOCK:
            case COLOR_CODE_BLOCK:
                if (requestTag == RequestTag.NORMAL_BLOCK && length != 10 ||
                    requestTag == RequestTag.COLOR_CODE_BLOCK && length != 12)
                {
                    throw new IllegalStateException(String.format("Unexpected data length %d in %s.",
                        length, requestTag));
                }
                else
                {
                    int index;
                    int runningChecksum = 0;

                    index = 0;
                    word = TrcUtil.bytesToInt(data[index], data[index + 1]);
                    runningChecksum += word;
                    currBlock.signature = word;

                    index += 2;
                    word = TrcUtil.bytesToInt(data[index], data[index + 1]);
                    runningChecksum += word;
                    currBlock.xCenter = word;

                    index += 2;
                    word = TrcUtil.bytesToInt(data[index], data[index + 1]);
                    runningChecksum += word;
                    currBlock.yCenter = word;

                    index += 2;
                    word = TrcUtil.bytesToInt(data[index], data[index + 1]);
                    runningChecksum += word;
                    currBlock.width = word;

                    index += 2;
                    word = TrcUtil.bytesToInt(data[index], data[index + 1]);
                    runningChecksum += word;
                    currBlock.height = word;

                    if (requestTag == RequestTag.COLOR_CODE_BLOCK)
                    {
                        index += 2;
                        word = TrcUtil.bytesToInt(data[index], data[index + 1]);
                        runningChecksum += word;
                        currBlock.angle = word;
                    }

                    if (runningChecksum == currBlock.checksum)
                    {
                        objects.add(currBlock);
                        currBlock = null;
                    }
                    asyncReadData(RequestTag.SYNC, 2);
                }
                break;

            default:
                throw new IllegalStateException(String.format("Unexpected request tag %s.", requestTag));
        }
    }   //processData

    //
    // Implements TrcDeviceQueue.CompletionHandler interface.
    //

    /**
     * This method is called when the read operation has been completed.
     *
     * @param requestTag specifies the tag to identify the request. Can be null if none was provided.
     * @param address specifies the data address read from if any, can be -1 if none specified.
     * @param data specifies the byte array containing data read.
     * @param error specifies true if the request failed, false otherwise. When true, data is invalid.
     */
    @Override
    public void readCompletion(Object requestTag, int address, byte[] data, boolean error)
    {
        final String funcName = "readCompletion";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.CALLBK, "tag=%s,addr=0x%x,data=%s,error=%s",
                requestTag, address, data != null? Arrays.toString(data): "null", Boolean.toString(error));
        }

        if (address == -1 && !error && data != null)
        {
            processData((RequestTag)requestTag, data, data.length);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.CALLBK);
        }
    }   //readCompletion

    /**
     * This method is called when the write operation has been completed.
     *
     * @param requestTag specifies the tag to identify the request. Can be null if none was provided.
     * @param address specifies the data address wrote to if any, can be -1 if none specified.
     * @param length specifies the number of bytes written.
     * @param error specifies true if the request failed to write the specified length, false otherwise.
     *              When true, length is invalid.
     */
    @Override
    public void writeCompletion(Object requestTag, int address, int length, boolean error)
    {
    }   //writeCompletion

}   //class FrcPixyCam
