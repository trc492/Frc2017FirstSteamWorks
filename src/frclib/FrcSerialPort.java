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

import java.util.Arrays;
import java.util.concurrent.ConcurrentLinkedQueue;

import edu.wpi.first.wpilibj.SerialPort;
import trclib.TrcDbgTrace;
import trclib.TrcEvent;
import trclib.TrcUtil;

/**
 * This class implements a platform dependent serial port device. It extends the WPILib SerialPort class and creates
 * a separate thread to provide asynchronous access to the device.
 *
 * @param <T> specifies the request ID object type.
 */
public class FrcSerialPort extends SerialPort implements Runnable
{
    private static final String moduleName = "FrcSerialPort";
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
         * @param requestId specifies the request ID. Can be null if none was specified.
         * @param data specifies the byte array read.
         */
        void readCompletion(Object requestId, byte[] data);

        /**
         * This method is called when the write operation has been completed.
         *
         * @param requestId specifies the request ID. Can be null if none was specified.
         * @param length specifies the number of bytes written.
         */
        void writeCompletion(Object requestId, int length);

    }   //interface CompletionHandler

    /**
     * This class implements a serial port request. Typically, a request will be put in the request queue so that
     * each request will be sent to the serial port device sequentially.
     */
    private class Request
    {
        public Object requestId;
        public byte[] buffer;
        public int length;
        public boolean repeat;
        public TrcEvent event;
        public CompletionHandler handler;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param requestId specifies the request ID that will be passed to the completion handler. It can be null.
         * @param buffer specifies the data buffer that contains data for a write request. This is null if it is a
         *        read request.
         * @param length specifies the number of bytes to read or write.
         * @param repeat specifies true to re-queue the request when completed.
         * @param event specifies the event to signal when the request is completed. It can be null.
         * @param handler specifies the completion handler to call when the request is completed. It can be null.
         */
        public Request(
            Object requestId, byte[] buffer, int length, boolean repeat, TrcEvent event, CompletionHandler handler)
        {
            this.requestId = requestId;
            this.buffer = buffer;
            this.length = length;
            this.repeat = repeat;
            this.event = event;
            this.handler = handler;
        }   //Request

    }   //class Request

    private boolean perfReportEnabled = false;
    private TrcDbgTrace tracer = null;
    private double totalTime = 0.0;
    private int totalRequests = 0;

    private final String instanceName;
    private ConcurrentLinkedQueue<Request> requestQueue;
    private Thread deviceTask;
    private volatile long processingInterval = 0;    // in msec

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the serial port (on-board or on the MXP).
     * @param baudRate specifies the serial baud rate.
     * @param dataBits specifies the number of data bits.
     * @param parity specifies the parity type.
     * @param stopBits specifies the number of stop bits.
     */
    public FrcSerialPort(
        final String instanceName, Port port, int baudRate, int dataBits, Parity parity, StopBits stopBits)
    {
        super(baudRate, port, dataBits, parity, stopBits);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        requestQueue = new ConcurrentLinkedQueue<>();
        deviceTask = new Thread(this, instanceName);
    }   //FrcSerialPort

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param baudRate specifies the serial baud rate.
     * @param dataBits specifies the number of data bits.
     * @param parity specifies the parity type.
     * @param stopBits specifies the number of stop bits.
     */
    public FrcSerialPort(
        final String instanceName, int baudRate, int dataBits, Parity parity, StopBits stopBits)
    {
        this(instanceName, Port.kOnboard, baudRate, dataBits, parity, stopBits);
    }   //FrcSerialPort

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param port specifies the serial port (on-board or on the MXP).
     * @param baudRate specifies the serial baud rate.
     */
    public FrcSerialPort(
        final String instanceName, Port port, int baudRate)
    {
        this(instanceName, port, baudRate, 8, Parity.kNone, StopBits.kOne);
    }   //FrcSerialPort

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
     * This method checks if the periodic task is enabled.
     *
     * @return true if task is enabled, false otherwise.
     */
    public synchronized boolean isTaskEnabled()
    {
        return deviceTask.isAlive();
    }   //isTaskEnabled

    /**
     * This method enables/disables the periodic task. If enabling task, the task will be started. If disabling task,
     * the task will be terminated. I
     *
     * @param enabled specifies true to enable periodic task, false to disable.
     */
    public synchronized void setTaskEnabled(boolean enabled)
    {
        boolean isAlive = deviceTask.isAlive();

        if (!isAlive && enabled)
        {
            totalTime = 0.0;
            totalRequests = 0;
            deviceTask.start();
        }
        else if (isAlive && !enabled)
        {
            deviceTask.interrupt();
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
     * This method is doing a synchronous read from the device with the specified length to read.
     *
     * @param length specifies the number of bytes to read.
     * @return data read as an array of bytes.
     */
    public byte[] syncRead(int length)
    {
        final String funcName = "syncRead";
        byte[] data = null;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "len=%d", length);
        }

        if (!isTaskEnabled())
        {
            throw new RuntimeException("Must call setTaskEnabled first.");
        }

        TrcEvent event = new TrcEvent(instanceName + "." + funcName + "." + length);
        Request request = new Request(null, null, length, false, event, null);

        requestQueue.add(request);

        while (!event.isSignaled())
        {
            Thread.yield();
        }

        data = request.buffer;
        request.buffer = null;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s",
                data == null? "null": Arrays.toString(data));
        }

        return data;
    }   //syncRead

    /**
     * This method is doing a synchronous write to the device with the specified data and length.
     *
     * @param data specifies the data to write to the device.
     * @param length specifies the number of bytes to write.
     * @return number of bytes written.
     */
    public int syncWrite(byte[] data, int length)
    {
        final String funcName = "syncWrite";
        int bytesWritten;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "data=%s,length=%d",
                Arrays.toString(data), length);
        }

        if (!isTaskEnabled())
        {
            throw new RuntimeException("Must call setTaskEnabled first.");
        }

        TrcEvent event = new TrcEvent(instanceName + "." + funcName + "." + length);
        Request request = new Request(null, data, length, false, event, null);

        requestQueue.add(request);

        while (!event.isSignaled())
        {
            Thread.yield();
        }
        bytesWritten = request.length;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", bytesWritten);
        }

        return bytesWritten;
    }   //syncWrite

    /**
     * This method is doing an asynchronous read from the device with the specified length to read.
     *
     * @param requestId specifies the request ID that will be passed to the completion handler. It can be null.
     * @param length specifies the number of bytes to read.
     * @param repeat specifies true to re-queue the request when completed.
     * @param event specifies the event to signal when the request is completed. It can be null.
     * @param handler specifies the completion handler to call when the request is completed. It can be null.
     */
    public void asyncRead(Object requestId, int length, boolean repeat, TrcEvent event, CompletionHandler handler)
    {
        final String funcName = "asyncRead";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "id=%s,len=%d,repeat=%s,event=%s",
                requestId != null? requestId: "null", length, Boolean.toString(repeat),
                event == null? "null": event.toString());
        }

        requestQueue.add(new Request(requestId, null, length, repeat, event, handler));

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //asyncRead

    /**
     * This method is doing an asynchronous read from the device with the specified length to read.
     *
     * @param requestId specifies the request ID that will be passed to the completion handler. It can be null.
     * @param length specifies the number of bytes to read.
     * @param event specifies the event to signal when the request is completed. It can be null.
     * @param handler specifies the completion handler to call when the request is completed. It can be null.
     */
    public void asyncRead(Object requestId, int length, TrcEvent event, CompletionHandler handler)
    {
        asyncRead(requestId, length, false, event, handler);
    }   //asyncRead

    /**
     * This method is doing an asynchronous write to the device with the specified data and length
     *
     * @param requestId specifies the request ID that will be passed to the completion handler. It can be null.
     * @param data specifies the data to write to the device.
     * @param length specifies the number of bytes to write.
     * @param event specifies the event to signal when the request is completed. It can be null.
     * @param handler specifies the completion handler to call when the request is completed. It can be null.
     */
    public void asyncWrite(Object requestId, byte[] data, int length, TrcEvent event, CompletionHandler handler)
    {
        final String funcName = "asyncWrite";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "id=%s,data=%s,length=%d,event=%s",
                requestId != null? requestId: "null", Arrays.toString(data), length,
                event == null? "null": event.toString());
        }

        requestQueue.add(new Request(requestId, data, length, false, event, handler));

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //asyncWrite

    /**
     * This method sends a byte command to the device.
     *
     * @param command specifies the command byte.
     * @param waitForCompletion specifies true to wait for write completion.
     */
    public void sendByteCommand(int regAddress, byte command, boolean waitForCompletion)
    {
        final String funcName = "sendByteCommand";
        byte[] data = new byte[1];

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "cmd=%x,sync=%s",
                command, Boolean.toString(waitForCompletion));
        }

        data[0] = command;
        if (waitForCompletion)
        {
            syncWrite(data, data.length);
        }
        else
        {
            asyncWrite(null, data, data.length, null, null);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //sendByteCommand

    /**
     * This method sends a 16-bit command to the device.
     *
     * @param command specifies the 16-bit command.
     * @param waitForCompletion specifies true to wait for write completion.
     */
    public void sendWordCommand(short command, boolean waitForCompletion)
    {
        final String funcName = "sendWordCommand";
        byte[] data = new byte[2];

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "cmd=%x,sync=%s",
                command, Boolean.toString(waitForCompletion));
        }

        data[0] = (byte)(command & 0xff);
        data[1] = (byte)(command >> 8);
        if (waitForCompletion)
        {
            syncWrite(data, data.length);
        }
        else
        {
            asyncWrite(null, data, data.length, null, null);
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

        while (!Thread.interrupted())
        {
            long requestStartTime = TrcUtil.getCurrentTimeMillis();
            Request request = requestQueue.poll();

            if (request != null)
            {
                boolean readRequest;
                byte[] buffer = null;
                int length = 0;
                double startTime;
                double elapsedTime;

                startTime = TrcUtil.getCurrentTime();
                if (request.buffer == null)
                {
                    //
                    // Read request.
                    //
                    readRequest = true;
                    buffer = read(request.length);

                    if (request.repeat)
                    {
                        requestQueue.add(request);
                    }
                }
                else
                {
                    //
                    // Write request.
                    //
                    readRequest = false;
                    length = write(request.buffer, request.length);
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
                    if (readRequest)
                    {
                        request.handler.readCompletion(request.requestId, buffer);
                    }
                    else
                    {
                        request.handler.writeCompletion(request.requestId, length);
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

}   //class FtcSerialPort
