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

import org.opencv.core.Rect;

import trclib.TrcDbgTrace;
import trclib.TrcStateMachine;
import trclib.TrcUtil;

/**
 * This class implements a platform dependent pixy camera that extends FrcI2cDevice. It provides access to the last
 * detected objects reported by the pixy camera asynchronously.
 */
public class FrcPixyCam extends FrcI2cDevice implements FrcI2cDevice.CompletionHandler
{
    private static final String moduleName = "FrcI2cDevice";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    /**
     * This class implements the pixy camera object block communication protocol. 
     */
    private class ObjectBlock
    {
        public int sync;
        public int checksum;
        public int signature;
        public int xCenter;
        public int yCenter;
        public int width;
        public int height;
    }   //class ObjectBlock

    /**
     * This is used by the state machine to parse the pixy camera object block.
     */
    private static enum State
    {
        RECEIVE_SYNC,
        RECEIVE_CHECKSUM,
        RECEIVE_SIGNATURE,
        RECEIVE_XCENTER,
        RECEIVE_YCENTER,
        RECEIVE_WIDTH,
        RECEIVE_HEIGHT,
        END_FRAME
    }   //enum State

    private static final int DEF_I2C_ADDRESS = 0x34;

    private byte[] data = new byte[2];
    private ArrayList<ObjectBlock> objects = new ArrayList<>();
    private Rect[] detectedObjectRects = null;
    private TrcStateMachine<State> sm = new TrcStateMachine<>(moduleName);
    private ObjectBlock currBlock = null;
    private int currChecksum = 0;
    private Object objectLock = new Object();

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the I2C port on the RoboRIO.
     * @param devAddress specifies the I2C address of the device.
     */
    public FrcPixyCam(final String instanceName, Port port, int devAddress)
    {
        super(instanceName, port, devAddress);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        asyncRead(0, data, true, null, this);
        sm.start(State.RECEIVE_SYNC);
    }   //FrcPixyCam

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the I2C port on the RoboRIO.
     */
    public FrcPixyCam(final String instanceName, Port port)
    {
        this(instanceName, port, DEF_I2C_ADDRESS);
    }   //FrcPixyCam

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param devAddress specifies the I2C address of the device.
     */
    public FrcPixyCam(final String instanceName, int devAddress)
    {
        this(instanceName, Port.kOnboard, devAddress);
    }   //FrcPixyCam

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FrcPixyCam(final String instanceName)
    {
        this(instanceName, Port.kOnboard, DEF_I2C_ADDRESS);
    }   //FrcPixyCam

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
        data[1] = (byte)0xfe;
        data[2] = brightness;
        asyncWrite(0, data, null, null);

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
        data[1] = (byte)0xff;
        data[2] = (byte)(pan & 0xff);
        data[3] = (byte)(pan >> 8);
        data[4] = (byte)(tilt & 0xff);
        data[5] = (byte)(tilt >> 8);
        asyncWrite(0, data, null, null);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setPanTilt

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
        data[1] = (byte)0xfd;
        data[2] = red;
        data[3] = green;
        data[4] = blue;
        asyncWrite(0, data, null, null);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setLED

    /**
     * This method returns an array of rectangles of last detected objects.
     *
     * @return array of rectangle of last detected objects.
     */
    public Rect[] getObjectRects()
    {
        final String funcName = "getObjectRects";
        Rect[] objectRects = null;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        synchronized (objectLock)
        {
            objectRects = detectedObjectRects;
            detectedObjectRects = null;
        }

        return objectRects;
    }   //getObjectRects

    //
    // Implements FrcI2cDevice.CompletionHandler interface.
    //

    /**
     * This method is called when the read operation has been completed.
     *
     * @param regAddress specifies the starting register address.
     * @param length specifies the number of bytes read.
     * @param timestamp specified the timestamp of the data retrieved.
     * @param data specifies the data byte array.
     * @param error specifies true if the operation failed, false otherwise.
     */
    public void readCompletion(int regAddress, int length, double timestamp, byte[] data, boolean error)
    {
        if (regAddress == 0 && length == 2 && !error)
        {
            int word = TrcUtil.bytesToInt(data[0], data[1]);
            State state = sm.getState();

            switch (state)
            {
                case RECEIVE_SYNC:
                    if (word == 0xaa55 || word == 0xaa56)
                    {
                        currBlock = new ObjectBlock();
                        currBlock.sync = word;
                        sm.setState(State.RECEIVE_CHECKSUM);
                    }
                    break;

                case RECEIVE_CHECKSUM:
                    if (word == 0xaa55 || word == 0xaa56)
                    {
                        currBlock = null;
                        sm.setState(State.END_FRAME);
                    }
                    else
                    {
                        currBlock.checksum = word;
                        currChecksum = 0;
                        sm.setState(State.RECEIVE_SIGNATURE);
                    }
                    break;

                case RECEIVE_SIGNATURE:
                    if (word >= 0 && word < 8)
                    {
                        currChecksum += word;
                        currBlock.signature = word;
                        sm.setState(State.RECEIVE_XCENTER);
                    }
                    else
                    {
                        currBlock = null;
                        sm.setState(State.RECEIVE_SYNC);
                    }
                    break;

                case RECEIVE_XCENTER:
                    currChecksum += word;
                    currBlock.xCenter = word;
                    sm.setState(State.RECEIVE_YCENTER);
                    break;

                case RECEIVE_YCENTER:
                    currChecksum += word;
                    currBlock.yCenter = word;
                    sm.setState(State.RECEIVE_WIDTH);
                    break;

                case RECEIVE_WIDTH:
                    currChecksum += word;
                    currBlock.width = word;
                    sm.setState(State.RECEIVE_HEIGHT);
                    break;

                case RECEIVE_HEIGHT:
                    currChecksum += word;
                    currBlock.height = word;
                    if (currChecksum == currBlock.checksum)
                    {
                        objects.add(currBlock);
                    }
                    currBlock = null;
                    sm.setState(State.RECEIVE_SYNC);
                    break;

                case END_FRAME:
                    if (objects.size() > 0)
                    {
                        synchronized (objectLock)
                        {
                            detectedObjectRects = new Rect[objects.size()];
                            for (int i = 0; i < objects.size(); i++)
                            {
                                ObjectBlock objBlock = objects.get(i);
                                detectedObjectRects[i].x = objBlock.xCenter - objBlock.width/2;
                                detectedObjectRects[i].y = objBlock.yCenter - objBlock.height/2;
                                detectedObjectRects[i].width = objBlock.width;
                                detectedObjectRects[i].height = objBlock.height;
                            }
                            objects.clear();
                        }
                    }
                    sm.setState(State.RECEIVE_SYNC);
                    break;
            }
        }
    }   //readCompletion

    /**
     * This method is called when the write operation has been completed (not implemented).
     *
     * @param regAddress specifies the starting register address.
     * @param length specifies the number of bytes read.
     * @param error specifies true if the operation failed, false otherwise.
     */
    public void writeCompletion(int regAddress, int length, boolean error)
    {
    }   //writeCompletion

}   //class FrcPixyCam
