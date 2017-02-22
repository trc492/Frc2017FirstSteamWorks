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

    public static final int DEF_I2C_ADDRESS = 0x54;

    private static final int PIXY_START_WORD                    = 0xaa55;
    private static final int PIXY_START_WORD_CC                 = 0xaa56;
    private static final int PIXY_START_WORDX                   = 0x55aa;
    private static final byte PIXY_ORPHAN_BYTE                  = (byte)0xaa;
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
    }   //class ObjectBlock

    /**
     * This is used by the state machine to parse the pixy camera object block.
     */
    private static enum State
    {
        SYNC,
        CHECKSUM,
        SIGNATURE,
        XCENTER,
        YCENTER,
        WIDTH,
        HEIGHT
    }   //enum State

    private byte[] data = new byte[2];
    private ArrayList<ObjectBlock> objects = new ArrayList<>();
    private ObjectBlock[] detectedObjects = null;
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
     * This method enables/disables the pixy camera task.
     *
     * @param enabled specifies true to enable pixy camera, false to disable.
     */
    public void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enanbled=%s", Boolean.toString(enabled));
        }

        super.setTaskEnabled(enabled);
        if (enabled)
        {
            asyncRead(data, data.length, false, null, this);
            sm.start(State.SYNC);
        }
        else
        {
            sm.stop();
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setEnabled

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
        asyncWrite(data, data.length, null, null);

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
        asyncWrite(data, data.length, null, null);

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
        asyncWrite(data, data.length, null, null);

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
        if (regAddress == -1 && !error)
        {
            if (length == 1)
            {
                //
                // We were not word aligned, we should be now.
                //
                if (data[0] == PIXY_ORPHAN_BYTE)
                {
                    //
                    // Detected a normal block.
                    //
                    currBlock = new ObjectBlock();
                    currBlock.sync = PIXY_START_WORD;
                    sm.setState(State.CHECKSUM);
                }
                else
                {
                    //
                    // Something still not right, wait for another sync word.
                    //
                    sm.setState(State.SYNC);
                }
            }
            else if (length == 2)
            {
                int word = TrcUtil.bytesToInt(data[0], data[1]);
                State state = sm.getState();

                switch (state)
                {
                    case SYNC:
                        if (word == PIXY_START_WORDX)
                        {
                            //
                            // We are not word aligned, consume the next byte.
                            //
                            asyncRead(data, 1, false, null, this);
                        }
                        else if (word == PIXY_START_WORD || word == PIXY_START_WORD_CC)
                        {
                            currBlock = new ObjectBlock();
                            currBlock.sync = word;
                            sm.setState(State.CHECKSUM);
                        }
                        break;

                    case CHECKSUM:
                        if (word != PIXY_START_WORD && word != PIXY_START_WORD_CC)
                        {
                            currBlock.checksum = word;
                            currChecksum = 0;
                            sm.setState(State.SIGNATURE);
                        }
                        else
                        {
                            //
                            // Detected end-of-frame, dispose the empty currBlock we just allocated and process end
                            // of frame.
                            //
                            currBlock = null;
                            if (objects.size() > 0)
                            {
                                synchronized (objectLock)
                                {
                                    detectedObjects = (ObjectBlock[])objects.toArray();
                                    objects.clear();
                                }
                            }
                            sm.setState(State.SYNC);
                        }
                        break;

                    case SIGNATURE:
                        currChecksum += word;
                        currBlock.signature = word;
                        sm.setState(State.XCENTER);
                        break;

                    case XCENTER:
                        currChecksum += word;
                        currBlock.xCenter = word;
                        sm.setState(State.YCENTER);
                        break;

                    case YCENTER:
                        currChecksum += word;
                        currBlock.yCenter = word;
                        sm.setState(State.WIDTH);
                        break;

                    case WIDTH:
                        currChecksum += word;
                        currBlock.width = word;
                        sm.setState(State.HEIGHT);
                        break;

                    case HEIGHT:
                        currChecksum += word;
                        currBlock.height = word;
                        if (currChecksum == currBlock.checksum)
                        {
                            objects.add(currBlock);
                        }
                        currBlock = null;
                        sm.setState(State.SYNC);
                        break;
                }
            }
            //
            // Read the next word.
            //
            asyncRead(data, 2, false, null, this);
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
