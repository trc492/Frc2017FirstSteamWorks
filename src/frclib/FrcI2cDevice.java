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
import trclib.TrcDbgTrace;
import trclib.TrcEvent;
import trclib.TrcUtil;

/**
 * This class implements a platform dependent I2C device.
 */
public class FrcI2cDevice extends I2C implements Runnable
{
    private static final String moduleName = "FrcI2cDevice";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    /**
     * This class implements an I2C device request. Typically, a request will be put in a bus transaction queue so
     * that each request will be sent to the I2C device sequentially.
     */
    private class Request
    {
        public int regAddress;
        public byte[] buffer;
        public boolean readRequest;
        public boolean repeat;
        public TrcEvent completionEvent;
        public boolean error;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param regAddress specifies the register address.
         * @param buffer specifies the data buffer that contains data for a write request or to be filled with data
         *               for a read request. The requested length is implied by the length of the buffer.
         * @param readRequest specifies true for a read request, false for a write request.
         * @param repeat specifies true to re-queue the request when completed.
         * @param event specifies the event to signal when the request is completed. It can be null.
         */
        public Request(int regAddress, byte[] buffer, boolean readRequest, boolean repeat, TrcEvent event)
        {
            this.regAddress = regAddress;
            this.buffer = buffer;
            this.readRequest = readRequest;
            this.repeat = repeat;
            this.completionEvent = event;
            this.error = false;
        }   //Request

//        public boolean isIntersectedOrAdjacentWith(int startAddress, int length)
//        {
//            int requestEnd = regAddress + buffer.length;
//            int endAddress = startAddress + length;
//
//            return startAddress >= regAddress && startAddress <= requestEnd
//                || endAddress >= regAddress && endAddress <= requestEnd
//                || regAddress >= startAddress && regAddress <= endAddress
//                || requestEnd >= startAddress && requestEnd <= endAddress;
//        }   //isIntersectedOrAdjacentWith
//
//        public boolean coalesceRange(int startAddress, int length)
//        {
//            boolean coalesce = isIntersectedOrAdjacentWith(startAddress, length);
//
//            if (coalesce)
//            {
//                int requestEnd = regAddress + this.length;
//                int endAddress = startAddress + length;
//
//                regAddress = Math.min(regAddress, startAddress);
//                requestEnd = Math.max(requestEnd, endAddress);
//                this.length = requestEnd - regAddress;
//            }
//
//            return coalesce;
//        }   //coalesceRange

    }   //class Request

    private boolean perfReportEnabled = false;
    private TrcDbgTrace tracer = null;
    private double totalTime = 0.0;
    private int totalRequests = 0;

    private final String instanceName;
    private ConcurrentLinkedQueue<Request> requestQueue;
    private Thread deviceTask;
    private volatile boolean taskTerminated = false;
    private volatile boolean taskEnabled = false;
    private volatile long processingInterval = 0;    // in msec

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the I2C port the device is connected to.
     * @param devAddress specifies the address of the device on the I2C bus.
     */
    public FrcI2cDevice(final String instanceName, Port port, int devAddress)
    {
        super(port, devAddress);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        requestQueue = new ConcurrentLinkedQueue<>();
        deviceTask = new Thread(this, instanceName);
        deviceTask.start();
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

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method checks if the periodic task has been terminated.
     *
     * @return true if task has been terminated, false otherwise.
     */
    public synchronized boolean isTaskTerminated()
    {
        return taskTerminated;
    }   //isTaskTerminated

    /**
     * This method is called to terminate the periodic task.
     */
    public synchronized void terminateTask()
    {
        taskTerminated = true;
    }   //terminateTask

    /**
     * This method checks if the periodic task is enabled.
     *
     * @return true if task is enabled, false otherwise.
     */
    public synchronized boolean isTaskEnabled()
    {
        return !taskTerminated && taskEnabled;
    }   //isTaskEnabled

    /**
     * This method enables/disables the periodic task. If this is called to disable the task, the task will be
     * set to a paused state. The operation will be resumed when this is called to enable it again.
     *
     * @param enabled specifies true to enable periodic task, false to disable.
     */
    public synchronized void setTaskEnabled(boolean enabled)
    {
        if (!taskTerminated)
        {
            taskEnabled = enabled;
        }
    }   //setTaskEnabled

    /**
     * This method sets the periodic task processing interval.
     *
     * @param interval specifies the processing interval in msec. If 0, process as fast as the CPU can run.
     */
    public synchronized void setProcessingInterval(long interval)
    {
        final String funcName = "setProcessingInterval";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "interval=%dms", interval);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        processingInterval = interval;
    }   //setProcessInterval

    /**
     * This method returns the periodic task processing interval.
     *
     * @return periodic task processing interval in msec.
     */
    public synchronized long getProcessingInterval()
    {
        final String funcName = "getProcessingInterval";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", processingInterval);
        }

        return processingInterval;
    }   //getProcessingInterval

    /**
     * This method enables/disables vision processing performance report.
     *
     * @param enabled specifies true to enable performance report, false to disable.
     */
    public synchronized void setPerfReportEnabled(boolean enabled)
    {
        perfReportEnabled = enabled;
        if (perfReportEnabled && tracer == null)
        {
            tracer = FrcRobotBase.getGlobalTracer();
        }
    }   //setPerfReportEnabled

    /**
     * This method check if the I2C device is enabled.
     *
     * @return true if the device state indicates it is enabled, false otherwise.
     */
    public synchronized boolean isDeviceEnabled()
    {
        final String funcName = "isDeviceEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(taskEnabled));
        }

        return taskEnabled;
    }   //isDeviceEnabled

    /**
     * This method is called to enable/disable the I2C device so that it will not unnecessarily bog down the I2C bus
     * bandwidth if it is not needed.
     *
     * @param enabled specifies true to enable device, false otherwise.
     */
    public synchronized void setDeviceEnabled(boolean enabled)
    {
        final String funcName = "setDeviceEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        taskEnabled = enabled;
        if (enabled)
        {
            totalTime = 0.0;
            totalRequests = 0;
        }
    }   //setDeviceEnabled

    /**
     * This method is doing a synchronous read from the device with the specified starting address and length of the
     * register block.
     *
     * @param startAddress specifies the starting register to read from.
     * @param buffer specifies the buffer to hold the data read.
     * @return true if read was successful, false otherwise.
     */
    public boolean syncRead(int startAddress, byte[] buffer)
    {
        final String funcName = "syncRead";
        boolean success = false;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "start=0x%02x,len=%d",
                startAddress, buffer.length);
        }

        if (!isDeviceEnabled())
        {
            throw new RuntimeException("Must call setDeviceEnabled first.");
        }

        TrcEvent event = new TrcEvent(instanceName + "." + startAddress + "." + buffer.length);
        Request request = new Request(startAddress, buffer, true, false, event);

        requestQueue.add(request);
        while (!event.isSignaled())
        {
            Thread.yield();
        }

        if (!request.error)
        {
            success = true;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s (%s)",
                Boolean.toString(success), Arrays.toString(buffer));
        }

        return success;
    }   //syncRead

    /**
     * This method is doing a synchronous write to the device with the specified starting address and data of the
     * register block.
     *
     * @param startAddress specifies the starting register to read from.
     * @param data specifies the data to write to the device.
     * @return true if write was successful, false otherwise.
     */
    public boolean syncWrite(int startAddress, byte[] data)
    {
        final String funcName = "syncWrite";
        boolean success = false;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "start=0x%02x,data=%s",
                startAddress, Arrays.toString(data));
        }

        if (!isDeviceEnabled())
        {
            throw new RuntimeException("Must call setDeviceEnabled first.");
        }

        TrcEvent event = new TrcEvent(instanceName + "." + startAddress + "." + data.length);
        Request request = new Request(startAddress, data, false, false, event);

        requestQueue.add(request);
        while (!event.isSignaled())
        {
            Thread.yield();
        }

        if (!request.error)
        {
            success = true;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(success));
        }

        return success;
    }   //syncWrite

    /**
     * This method is doing an asynchronous read from the device with the specified starting address and length of the
     * register block.
     *
     * @param startAddress specifies the starting register to read from.
     * @param buffer specifies the buffer to hold the data read.
     * @param repeat specifies true to re-queue the request when completed.
     * @param event specifies the event to signal when the request is completed. It can be null.
     */
    public void asyncRead(int startAddress, byte[] buffer, boolean repeat, TrcEvent event)
    {
        final String funcName = "asyncRead";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "start=0x%02x,len=%d,repeat=%s,event=%s",
                startAddress, buffer.length, Boolean.toString(repeat), event.toString());
        }

        if (!isDeviceEnabled())
        {
            throw new RuntimeException("Must enable device first.");
        }

        requestQueue.add(new Request(startAddress, buffer, true, repeat, event));

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //asyncRead

    /**
     * This method is doing an asynchronous read from the device with the specified starting address and length of the
     * register block.
     *
     * @param startAddress specifies the starting register to read from.
     * @param buffer specifies the buffer to hold the data read.
     * @param event specifies the event to signal when the request is completed. It can be null.
     */
    public void asyncRead(int startAddress, byte[] buffer, TrcEvent event)
    {
        asyncRead(startAddress, buffer, false, event);
    }   //asyncRead

    /**
     * This method is doing an asynchronous write to the device with the specified starting address of the register
     * block and data buffer.
     *
     * @param startAddress specifies the starting register to read from.
     * @param data specifies the data to write to the device.
     * @param event specifies the event to signal when the request is completed. It can be null.
     */
    public void asyncWrite(int startAddress, byte[] data, TrcEvent event)
    {
        final String funcName = "asyncWrite";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "start=0x%02x,data=%s,event=%s",
                startAddress, Arrays.toString(data), event.toString());
        }

        if (!isDeviceEnabled())
        {
            throw new RuntimeException("Must enable device first.");
        }

        requestQueue.add(new Request(startAddress, data, false, false, event));

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //asyncWrite

    /**
     * This method sends a byte command to the device.
     *
     * @param regAddress specifies the register address to write to.
     * @param command specifies the command byte.
     * @param waitForCompletion specifies true to wait for write completion.
     */
    public void sendByteCommand(int regAddress, byte command, boolean waitForCompletion)
    {
        final String funcName = "sendByteCommand";
        byte[] data = new byte[1];

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "addr=%x,cmd=%x,sync=%s",
                regAddress, command, Boolean.toString(waitForCompletion));
        }

        data[0] = command;
        if (waitForCompletion)
        {
            syncWrite(regAddress, data);
        }
        else
        {
            asyncWrite(regAddress, data, null);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //sendByteCommand

    /**
     * This method sends a 16-bit command to the device.
     *
     * @param regAddress specifies the register address to write to.
     * @param command specifies the 16-bit command.
     * @param waitForCompletion specifies true to wait for write completion.
     */
    public void sendWordCommand(int regAddress, short command, boolean waitForCompletion)
    {
        final String funcName = "sendWordCommand";
        byte[] data = new byte[2];

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "addr=%x,cmd=%x,sync=%s",
                regAddress, command, Boolean.toString(waitForCompletion));
        }

        data[0] = (byte)(command & 0xff);
        data[1] = (byte)(command >> 8);
        if (waitForCompletion)
        {
            syncWrite(regAddress, data);
        }
        else
        {
            asyncWrite(regAddress, data, null);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //sendWordCommand

    //
    // Implements Runnable interface.
    //

    /**
     * This method runs the periodic processing task.
     */
    @Override
    public void run()
    {
        final String funcName = "run";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.CALLBK);
        }

        while (!Thread.interrupted() && !taskTerminated)
        {
            long requestStartTime = TrcUtil.getCurrentTimeMillis();

            if (isTaskEnabled())
            {
                Request request = requestQueue.poll();

                if (request != null)
                {
                    double startTime;
                    double elapsedTime;

                    startTime = TrcUtil.getCurrentTime();
                    if (request.readRequest)
                    {
                        if (!read(request.regAddress, request.buffer.length, request.buffer))
                        {
                            request.error = true;
                        }

                        if (request.repeat)
                        {
                            requestQueue.add(request);
                        }
                    }
                    else
                    {
                        ByteBuffer buffer = ByteBuffer.allocateDirect(request.buffer.length + 1);

                        buffer.put((byte)request.regAddress);
                        buffer.put(request.buffer);
                        if (!writeBulk(buffer, request.buffer.length + 1))
                        {
                            request.error = true;
                        }
                    }
                    elapsedTime = TrcUtil.getCurrentTime() - startTime;
                    totalTime += elapsedTime;
                    totalRequests++;
                    if (perfReportEnabled && tracer != null)
                    {
                        tracer.traceInfo(funcName, "Average request time = %.3f msec", totalTime/totalRequests);
                    }

                    if (request.completionEvent != null)
                    {
                        request.completionEvent.set(true);
                    }
                }
            }

            if (processingInterval > 0)
            {
                long sleepTime = processingInterval - (TrcUtil.getCurrentTimeMillis() - requestStartTime);
                TrcUtil.sleep(sleepTime);
            }
            else
            {
                Thread.yield();
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.CALLBK);
        }
    }   //run

}   //class FtcI2cDevice
