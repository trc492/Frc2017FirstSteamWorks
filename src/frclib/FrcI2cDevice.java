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
 * This class implements a platform dependent I2C device. It extends the WPILib I2C class and creates a separate
 * thread to provide asynchronous access to the device.
 */
public abstract class FrcI2cDevice extends I2C implements Runnable
{
    private static final String moduleName = "FrcI2cDevice";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    /**
     * This interface provides callback notification on asynchronous read/write completion.
     */
    public interface CompletionHandler
    {
        /**
         * This method is called when the read operation has been completed.
         *
         * @param regAddress specifies the starting register address.
         * @param length specifies the number of bytes read.
         * @param timestamp specified the timestamp of the data retrieved.
         * @param data specifies the data byte array.
         * @param error specifies true if the operation failed, false otherwise.
         */
        void readCompletion(int regAddress, int length, double timestamp, byte[] data, boolean error);

        /**
         * This method is called when the write operation has been completed.
         *
         * @param regAddress specifies the starting register address.
         * @param length specifies the number of bytes read.
         * @param error specifies true if the operation failed, false otherwise.
         */
        void writeCompletion(int regAddress, int length, boolean error);
    }   //interface CompletionHandler

    /**
     * This class implements an I2C device request. Typically, a request will be put in the request queue so that
     * each request will be sent to the I2C device sequentially.
     */
    private class Request
    {
        public int regAddress;
        public byte[] buffer;
        public int length;
        public boolean readRequest;
        public boolean repeat;
        public TrcEvent event;
        public CompletionHandler handler;
        public boolean error;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param regAddress specifies the register address.
         * @param buffer specifies the data buffer that contains data for a write request or to be filled with data
         *               for a read request. The requested length is implied by the length of the buffer.
         * @param length specifies the length of the register block to read/write.
         * @param readRequest specifies true for a read request, false for a write request.
         * @param repeat specifies true to re-queue the request when completed.
         * @param event specifies the event to signal when the request is completed. It can be null.
         * @param handler specifies the completion handler to call when the request is completed. It can be null.
         */
        public Request(
            int regAddress, byte[] buffer, int length, boolean readRequest, boolean repeat, TrcEvent event,
            CompletionHandler handler)
        {
            this.regAddress = regAddress;
            this.buffer = buffer;
            this.length = length;
            this.readRequest = readRequest;
            this.repeat = repeat;
            this.event = event;
            this.handler = handler;
            this.error = false;
        }   //Request

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
     * @param startAddress specifies the starting register to read from or -1 for read-only request.
     * @param buffer specifies the buffer to hold the data read.
     * @param length specifies the length of the register block to read.
     * @return true if read was successful, false otherwise.
     */
    public boolean syncRead(int startAddress, byte[] buffer, int length)
    {
        final String funcName = "syncRead";
        boolean success = false;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "start=0x%02x,len=%d", startAddress, length);
        }

        if (!isDeviceEnabled())
        {
            throw new RuntimeException("Must call setDeviceEnabled first.");
        }

        TrcEvent event = new TrcEvent(instanceName + "." + startAddress + "." + length);
        Request request = new Request(startAddress, buffer, length, true, false, event, null);

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
     * This method is doing a synchronous read-only from the device with the specified length.
     *
     * @param buffer specifies the buffer to hold the data read.
     * @param length specifies the length of the register block to read.
     * @return true if read was successful, false otherwise.
     */
    public boolean syncRead(byte[] buffer, int length)
    {
        return syncRead(-1, buffer, length);
    }   //syncRead

    /**
     * This method is doing a synchronous write to the device with the specified starting address and data of the
     * register block.
     *
     * @param startAddress specifies the starting register to write to or -1 for bulk write.
     * @param data specifies the data to write to the device.
     * @param length specifies the length of the register block to write.
     * @return true if write was successful, false otherwise.
     */
    public boolean syncWrite(int startAddress, byte[] data, int length)
    {
        final String funcName = "syncWrite";
        boolean success = false;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "start=0x%02x,data=%s,length=%d",
                startAddress, Arrays.toString(data), length);
        }

        if (!isDeviceEnabled())
        {
            throw new RuntimeException("Must call setDeviceEnabled first.");
        }

        TrcEvent event = new TrcEvent(instanceName + "." + startAddress + "." + data.length);
        Request request = new Request(startAddress, data, length, false, false, event, null);

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
     * This method is doing a synchronous bulk write to the device with the specified length.
     *
     * @param data specifies the data to write to the device.
     * @param length specifies the length of the register block to write.
     * @return true if write was successful, false otherwise.
     */
    public boolean syncWrite(byte[] data, int length)
    {
        return syncWrite(-1, data, length);
    }   //syncWrite

    /**
     * This method is doing an asynchronous read from the device with the specified starting address and length of the
     * register block.
     *
     * @param startAddress specifies the starting register to read from.
     * @param buffer specifies the buffer to hold the data read.
     * @param length specifies the length of the register block to read.
     * @param repeat specifies true to re-queue the request when completed.
     * @param event specifies the event to signal when the request is completed. It can be null.
     * @param handler specifies the completion handler to call when the request is completed. It can be null.
     */
    public void asyncRead(
        int startAddress, byte[] buffer, int length, boolean repeat, TrcEvent event, CompletionHandler handler)
    {
        final String funcName = "asyncRead";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "start=0x%02x,len=%d,repeat=%s,event=%s",
                startAddress, length, Boolean.toString(repeat), event.toString());
        }

        requestQueue.add(new Request(startAddress, buffer, length, true, repeat, event, handler));

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
     * @param length specifies the length of the register block to read.
     * @param event specifies the event to signal when the request is completed. It can be null.
     * @param handler specifies the completion handler to call when the request is completed. It can be null.
     */
    public void asyncRead(int startAddress, byte[] buffer, int length, TrcEvent event, CompletionHandler handler)
    {
        asyncRead(startAddress, buffer, length, false, event, handler);
    }   //asyncRead

    /**
     * This method is doing an asynchronous read-only from the device with the specified length.
     *
     * @param buffer specifies the buffer to hold the data read.
     * @param length specifies the length of the register block to read.
     * @param repeat specifies true to re-queue the request when completed.
     * @param event specifies the event to signal when the request is completed. It can be null.
     * @param handler specifies the completion handler to call when the request is completed. It can be null.
     */
    public void asyncRead(byte[] buffer, int length, boolean repeat, TrcEvent event, CompletionHandler handler)
    {
        asyncRead(-1, buffer, length, repeat, event, handler);
    }   //asyncRead

    /**
     * This method is doing an asynchronous read-only from the device with the specified length.
     *
     * @param buffer specifies the buffer to hold the data read.
     * @param length specifies the length of the register block to read.
     * @param event specifies the event to signal when the request is completed. It can be null.
     * @param handler specifies the completion handler to call when the request is completed. It can be null.
     */
    public void asyncRead(byte[] buffer, int length, TrcEvent event, CompletionHandler handler)
    {
        asyncRead(-1, buffer, length, false, event, handler);
    }   //asyncRead

    /**
     * This method is doing an asynchronous write to the device with the specified starting address of the register
     * block and data buffer.
     *
     * @param startAddress specifies the starting register to write to.
     * @param data specifies the data to write to the device.
     * @param length specifies the length of the register block to write.
     * @param event specifies the event to signal when the request is completed. It can be null.
     * @param handler specifies the completion handler to call when the request is completed. It can be null.
     */
    public void asyncWrite(int startAddress, byte[] data, int length, TrcEvent event, CompletionHandler handler)
    {
        final String funcName = "asyncWrite";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "start=0x%02x,data=%s,length=%d,event=%s",
                startAddress, Arrays.toString(data), length, event.toString());
        }

        requestQueue.add(new Request(startAddress, data, length, false, false, event, handler));

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //asyncWrite

    /**
     * This method is doing an asynchronous bulk write to the device with the specified length.
     *
     * @param data specifies the data to write to the device.
     * @param length specifies the length of the register block to write.
     * @param event specifies the event to signal when the request is completed. It can be null.
     * @param handler specifies the completion handler to call when the request is completed. It can be null.
     */
    public void asyncWrite(byte[] data, int length, TrcEvent event, CompletionHandler handler)
    {
        asyncWrite(-1, data, length, event, handler);
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
            syncWrite(regAddress, data, data.length);
        }
        else
        {
            asyncWrite(regAddress, data, data.length, null, null);
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
            syncWrite(regAddress, data, data.length);
        }
        else
        {
            asyncWrite(regAddress, data, data.length, null, null);
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
                        if (request.regAddress == -1 && !readOnly(request.buffer, request.length) ||
                            !read(request.regAddress, request.length, request.buffer))
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
                        ByteBuffer buffer;

                        if (request.regAddress == -1)
                        {
                            buffer = ByteBuffer.allocateDirect(request.length);
                        }
                        else
                        {
                            buffer = ByteBuffer.allocateDirect(request.length + 1);
                            buffer.put((byte)request.regAddress);
                        }
                        buffer.put(request.buffer);

                        if (!writeBulk(buffer, request.length + 1))
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

                    if (request.event != null)
                    {
                        request.event.set(true);
                    }

                    if (request.handler != null)
                    {
                        if (request.readRequest)
                        {
                            request.handler.readCompletion(
                                request.regAddress, request.length, TrcUtil.getCurrentTime(), request.buffer,
                                request.error);
                        }
                        else
                        {
                            request.handler.writeCompletion(request.regAddress, request.length, request.error);
                        }
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
