/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

import org.opencv.core.Mat;
import org.opencv.core.MatOfRect;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * This class implements a platform independent vision thread. It grabs a frame from the video source, calls the
 * provided object detector to process the frame and overlays rectangles on the detected objects in the image.
 * This class is to be extended by a platform dependent vision processor who will provide the video input and
 * output. 
 */
public abstract class TrcVision implements Runnable
{
    private static final String moduleName = "TrcVision";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private static final boolean visionPerfEnabled = true;

    /**
     * This method is called to grab a frame from the video input.
     *
     * @param frame specifies the frame buffer to hold the captured image.
     * @return true if frame is successfully captured, false otherwise.
     */
    public abstract boolean grabFrame(Mat frame);

    /**
     * This method is called to render a frame to the video output.
     * 
     * @param frame specifies the frame to be rendered to the video output.
     */
    public abstract void putFrame(Mat frame);

    /**
     * This interface provides a method to process a video frame to detect objects.
     */
    public interface ObjectDetector
    {
        /**
         * This method is called to detect objects in the provided video frame.
         *
         * @param image specified the image to be processed.
         * @param objRects specifies the object rectangle array to hold the detected objects.
         * @return true if detected objects, false otherwise.
         */
        boolean detectObjects(Mat image, MatOfRect objRects);
    }   //interface ObjectDetector

    private ObjectDetector objectDetector;
    private Mat image;
    private MatOfRect objRects;
    private long totalTime = 0;
    private long totalFrames = 0;

    private Object monitor;
    private Thread visionThread = null;

    private long processingInterval = 50;   // in msec
    private boolean taskEnabled = false;
    private boolean oneShotEnabled = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param objectDetector specifies the object detector.
     */
    public TrcVision(ObjectDetector objectDetector)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        this.objectDetector = objectDetector;
        image = new Mat();
        objRects = new MatOfRect();
        monitor = new Object();
        visionThread = new Thread(this, "VisionTask");
        visionThread.setDaemon(true);
        visionThread.start();
    }   //TrcVision

    /**
     * This method enables/disables the vision processing task.
     *
     * @param enabled specifies true to enable vision task, false to disable.
     */
    public void setTaskEnabled(boolean enabled)
    {
        final String funcName = "setTaskEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
        }

        if (!taskEnabled && enabled)
        {
            //
            // Enable task.
            //
            synchronized(monitor)
            {
                taskEnabled = true;
                monitor.notify();
            }
        }
        else if (taskEnabled && !enabled)
        {
            //
            // Disable task.
            //
            taskEnabled = false;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setTaskEnabled

    /**
     * This method returns the state of the vision task.
     *
     * @return true if the vision task is enabled, false otherwise.
     */
    public boolean isTaskEnabled()
    {
        final String funcName = "isTaskEnabled";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(taskEnabled));
        }

        return taskEnabled;
    }   //isTaskEnabled

    /**
     * This method sets the vision task processing interval.
     *
     * @param interval specifies the processing interval in msec.
     */
    public void setProcessingInterval(long interval)
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
     * This method returns the vision task processing interval.
     *
     * @return vision task processing interval in msec.
     */
    public long getProcessingInterval()
    {
        final String funcName = "getProcessingInterval";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", processingInterval);
        }

        return processingInterval;
    }   //getProcessingPeriod

    /**
     * This method runs the vision processing task.
     */
    public void run()
    {
        while (true)
        {
            synchronized(monitor)
            {
                //
                // Wait until we are enabled.
                //
                while (!taskEnabled && !oneShotEnabled)
                {
                    try
                    {
                        monitor.wait();
                    }
                    catch (InterruptedException e)
                    {
                    }
                }
            }

            long startTime = TrcUtil.getCurrentTimeMillis();
            processImage();
            long sleepTime = processingInterval - (TrcUtil.getCurrentTimeMillis() - startTime);

            TrcUtil.sleep(sleepTime);
        }
    }   //run

    /**
     * This method grabs a new video frame and processes it.
     */
    private void processImage()
    {
        final String funcName = "processImage";
        double startTime;
        double elapsedTime;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
        //
        // Grab a frame from the camera and put it in the source mat. If there is an error notify the output.
        //
        if (grabFrame(image))
        {
            //
            // Capture an image and subject it for object detection. The object detector produces an array of
            // rectangles representing objects detected.
            //
            startTime = TrcUtil.getCurrentTimeMillis();
            objectDetector.detectObjects(image, objRects);
            elapsedTime = TrcUtil.getCurrentTimeMillis() - startTime;
            totalTime += elapsedTime;
            totalFrames++;
            if (visionPerfEnabled && dbgTrace != null)
            {
                dbgTrace.traceInfo(funcName, "Average processing time = %.3f msec", (double)totalTime/totalFrames);
            }
            //
            // Overlay a rectangle on each detected object.
            //
            Rect[] rects = objRects.toArray();
            for (Rect rect: rects)
            {
                //
                // Draw a rectangle around the detected object.
                //
                Imgproc.rectangle(
                    image, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height),
                    new Scalar(0, 255, 0));
            }
            putFrame(image);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //processImage

}   //class TrcVision
