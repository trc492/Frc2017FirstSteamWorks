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

package frclib;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import trclib.TrcDbgTrace;
import trclib.TrcVisionTask;

/**
 * This class implements a generic OpenCV detector. Typically, it is extended by a specific detector that provides
 * the algorithm to process an image for detecting objects using OpenCV APIs.
 *
 * @param <O> specifies the type of the detected objects.
 */
public abstract class FrcOpenCVDetector<O> implements TrcVisionTask.VisionProcessor<O>
{
    private static final String moduleName = "FrcOpenCVDetector";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private final String instanceName;
    private CvSink videoIn;
    private CvSource videoOut;
    private Mat image;
    private O[] detectedObjectBuffers = null;
    private TrcVisionTask<O> visionTask;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param videoIn specifies the video input stream.
     * @param videoOut specifies the video output stream.
     */
    public FrcOpenCVDetector(final String instanceName, CvSink videoIn, CvSource videoOut)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        this.videoIn = videoIn;
        this.videoOut = videoOut;

        image = new Mat();
    }   //FrcOpenCVDetector

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
     * This method provides the preallocated buffer array for storing detected objects.
     *
     * @param detectedObjectBuffers specifies the array of detected object buffers.
     */
    public void setPreallocatedObjectBuffers(O[] detectedObjectBuffers)
    {
        this.detectedObjectBuffers = detectedObjectBuffers;
    }   //setPreallocatedObjectBuffers

    /**
     * This method enables/disables the vision processing task.
     *
     * @param enabled specifies true to enable vision task, false to disable.
     */
    public void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
        }

        if (visionTask == null)
        {
            visionTask = new TrcVisionTask<>(this, image, detectedObjectBuffers);
        }

        visionTask.setTaskEnabled(enabled);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setEnabled

    //
    // Implements the FrcVisionTask.VisionProcesor interface.
    //

    /**
     * This method is called to grab an image frame from the video input.
     *
     * @param image specifies the frame buffer to hold the captured image.
     * @return true if frame is successfully captured, false otherwise.
     */
    @Override
    public boolean grabFrame(Mat image)
    {
        boolean success = false;

        synchronized (image)
        {
            success = videoIn.grabFrame(image) != 0;
        }

        return success;
    }   //grabFrame

    /**
     * This method is called to render an image to the video output and overlay detected objects on top of it.
     *
     * @param image specifies the frame to be rendered to the video output.
     * @param detectedObjectRects specifies the detected object rectangles.
     * @param color specifies the color of the rectangle outline.
     * @param thickness specifies the thickness of the rectangle outline.
     */
    @Override
    public void putFrame(Mat image, Rect[] detectedObjectRects, Scalar color, int thickness)
    {
        //
        // Overlay a rectangle on each detected object.
        //
        synchronized (image)
        {
            if (detectedObjectRects != null)
            {
                for (Rect r: detectedObjectRects)
                {
                    //
                    // Draw a rectangle around the detected object.
                    //
                    Imgproc.rectangle(
                        image, new Point(r.x, r.y), new Point(r.x + r.width, r.y + r.height), color, thickness);
                }
            }

            videoOut.putFrame(image);
        }
    }   //putFrame

}   //class FrcOpenCVDetector
