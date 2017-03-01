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

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.concurrent.ConcurrentLinkedQueue;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import trclib.TrcDbgTrace;
import trclib.TrcDeviceQueue;
import trclib.TrcEvent;

/**
 * This class implements a platform dependent I2C device. It extends TrcDeviceQueue to provide asynchronous request
 * queue support and creates the I2C device using WPILib.
 */
public class FrcI2cDevice extends TrcDeviceQueue //implements TrcDeviceQueue.CompletionHandler
{
    private static final String moduleName = "FrcI2cDevice";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private I2C device;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the I2C port the device is connected to.
     * @param devAddress specifies the address of the device on the I2C bus.
     */
    public FrcI2cDevice(final String instanceName, Port port, int devAddress)
    {
        super(instanceName);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        device = new I2C(port, devAddress);
    }   //FrcI2cDevice

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param devAddress specifies the address of the device on the I2C bus.
     */
    public FrcI2cDevice(final String instanceName, int devAddress)
    {
        this(instanceName, Port.kOnboard, devAddress);
    }   //FrcI2cDevice

    //
    // Implements TrcDeviceQueue abstract methods.
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
        final String funcName = "readData";
        byte[] buffer = new byte[length];

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.CALLBK, "addr=%d,len=%d", address, length);
        }

        if (address == -1 && device.readOnly(buffer, length) || device.read(address, length, buffer))
        {
            buffer = null;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.CALLBK, "=%s", Arrays.toString(buffer));
        }

        return buffer;
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
        final String funcName = "writeData";
        int buffLen = address == -1? length: length + 1;
        ByteBuffer byteBuffer = ByteBuffer.allocateDirect(buffLen);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.CALLBK, "addr=%d,data=%s,len=%d",
                address, Arrays.toString(buffer), length);
        }

        if (address != -1)
        {
            byteBuffer.put((byte)address);
        }
        byteBuffer.put(buffer);

        if (device.writeBulk(byteBuffer, length))
        {
            length = 0;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.CALLBK, "=%d", length);
        }

        return length;
    }   //writeData

    //
    // Overriding methods in TrcDeviceQueue to fix WPILib bugs in the I2C class.
    //

//    private class ParentRequest
//    {
//        public Object requestTag;
//        public int address;
//        public int length;
//        public TrcEvent event;
//        public TrcDeviceQueue.CompletionHandler handler;
//        public byte[] readBuffer;
//        public int byteCount;
//        public boolean error;
//
//        public ParentRequest(
//            Object requestTag, int address, int length, TrcEvent event, TrcDeviceQueue.CompletionHandler handler,
//            boolean readRequest)
//        {
//            this.requestTag = requestTag;
//            this.address = address;
//            this.length = length;
//            this.event = event;
//            this.handler = handler;
//            this.readBuffer = readRequest? new byte[length]: null;
//            byteCount = 0;
//            error = false;
//        }   //ParentRequest
//
//    }   //class ParentRequest
//
//    private ConcurrentLinkedQueue<ParentRequest> parentRequestQueue = new ConcurrentLinkedQueue<>();
//
//    /**
//     * This method is doing an asynchronous read from the device with the specified length to read.
//     *
//     * @param requestTag specifies the tag to identify the request. Can be null if none was provided.
//     * @param address specifies the data address if any, can be -1 if no address is required.
//     * @param length specifies the number of bytes to read.
//     * @param repeat specifies true to re-queue the request when completed.
//     * @param event specifies the event to signal when the request is completed, can be null if none specified.
//     * @param handler specifies the completion handler to call when the request is completed, can be null if none
//     *                specified.
//     */
//    @Override
//    public void asyncRead(
//        Object requestTag, int address, int length, boolean repeat, TrcEvent event, CompletionHandler handler)
//    {
//        final String funcName = "asyncRead";
//
//        if (debugEnabled)
//        {
//            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "tag=%s,addr=%d,len=%d,repeat=%s,event=%s",
//                requestTag != null? requestTag: "null", address, length, Boolean.toString(repeat),
//                event == null? "null": event.toString());
//        }
//
//        if (length > 0)
//        {
//            parentRequestQueue.add(new ParentRequest(requestTag, address, length, event, handler, true));
//            for (int i = 0; i < length; i++)
//            {
//                queueRequest(requestTag, true, address != -1? address + i: -1, null, 1, repeat, null, this);
//            }
//        }
//
//        if (debugEnabled)
//        {
//            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
//        }
//    }   //asyncRead
//
//    /**
//     * This method is doing an asynchronous write to the device with the specified data and length
//     *
//     * @param requestTag specifies the tag to identify the request. Can be null if none was provided.
//     * @param address specifies the data address if any, can be -1 if no address is required.
//     * @param data specifies the buffer containing the data to write to the device.
//     * @param length specifies the number of bytes to write.
//     * @param event specifies the event to signal when the request is completed, can be null if none specified.
//     * @param handler specifies the completion handler to call when the request is completed, can be null if none
//     *                specified.
//     */
//    @Override
//    public void asyncWrite(
//        Object requestTag, int address, byte[] data, int length, TrcEvent event, CompletionHandler handler)
//    {
//        final String funcName = "asyncWrite";
//
//        if (debugEnabled)
//        {
//            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "tag=%s,addr=%d,data=%s,length=%d,event=%s",
//                requestTag != null? requestTag: "null", address, Arrays.toString(data), length,
//                event == null? "null": event.toString());
//        }
//
//        if (length > 0)
//        {
//            parentRequestQueue.add(new ParentRequest(requestTag, address, length, event, handler, false));
//            for (int i = 0; i < length; i++)
//            {
//                byte[] buffer = new byte[1];
//                buffer[0] = data[i];
//                queueRequest(requestTag, false, address != -1? address + i: -1, buffer, 1, false, null, this);
//            }
//        }
//
//        if (debugEnabled)
//        {
//            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
//        }
//    }   //asyncWrite
//
//    /**
//     * This method is called when the read operation has been completed.
//     *
//     * @param requestTag specifies the tag to identify the request. Can be null if none was provided.
//     * @param address specifies the data address read from if any, can be -1 if none specified.
//     * @param data specifies the byte array containing data read.
//     * @param error specifies true if the request failed, false otherwise. When true, data is invalid.
//     */
//    public void readCompletion(Object requestTag, int address, byte[] data, boolean error)
//    {
//        synchronized (parentRequestQueue)
//        {
//            ParentRequest parentRequest = parentRequestQueue.peek();
//
//            if (requestTag != parentRequest.requestTag)
//            {
//                throw new IllegalStateException(String.format("Request tag %s mismatch (expected %s).",
//                    requestTag, parentRequest.requestTag));
//            }
//            else if (error)
//            {
//                parentRequest.error = true;
//                parentRequest.readBuffer[parentRequest.byteCount] = 0;
//                parentRequest.byteCount++;
//            }
//            else if (data.length != 1)
//            {
//                throw new IllegalStateException(String.format("Invalid data %s (length %d).",
//                    Arrays.toString(data), data.length));
//            }
//            else
//            {
//                parentRequest.readBuffer[parentRequest.byteCount] = data[0];
//                parentRequest.byteCount++;
//                if (parentRequest.byteCount == parentRequest.length)
//                {
//                    if (parentRequest.event != null)
//                    {
//                        parentRequest.event.set(true);
//                    }
//
//                    if (parentRequest.handler != null)
//                    {
//                        parentRequest.handler.readCompletion(
//                            requestTag, parentRequest.address, parentRequest.readBuffer, parentRequest.error);
//                    }
//
//                    parentRequestQueue.remove(parentRequest);
//                }
//            }
//        }
//    }   //readCompletion
//
//    /**
//     * This method is called when the write operation has been completed.
//     *
//     * @param requestTag specifies the tag to identify the request. Can be null if none was provided.
//     * @param address specifies the data address wrote to if any, can be -1 if none specified.
//     * @param length specifies the number of bytes written.
//     * @param error specifies true if the request failed to write the specified length, false otherwise.
//     *              When true, length is invalid.
//     */
//    public void writeCompletion(Object requestTag, int address, int length, boolean error)
//    {
//        synchronized (parentRequestQueue)
//        {
//            ParentRequest parentRequest = parentRequestQueue.peek();
//
//            if (requestTag != parentRequest.requestTag)
//            {
//                throw new IllegalStateException(String.format("Request tag %s mismatch (expected %s).",
//                    requestTag, parentRequest.requestTag));
//            }
//            else if (error)
//            {
//                parentRequest.error = true;
//                parentRequest.byteCount++;
//            }
//            else if (length != 1)
//            {
//                throw new IllegalStateException(String.format("Invalid data length %d.", length));
//            }
//            else
//            {
//                parentRequest.byteCount++;
//                if (parentRequest.byteCount == parentRequest.length)
//                {
//                    if (parentRequest.event != null)
//                    {
//                        parentRequest.event.set(true);
//                    }
//
//                    if (parentRequest.handler != null)
//                    {
//                        parentRequest.handler.writeCompletion(
//                            requestTag, parentRequest.address, parentRequest.length, parentRequest.error);
//                    }
//
//                    parentRequestQueue.remove(parentRequest);
//                }
//            }
//        }
//    }   //writeCompletion

}   //class FtcI2cDevice
