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
import edu.wpi.first.wpilibj.CameraServer;
import org.opencv.core.Mat;
import org.opencv.core.MatOfRect;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import trclib.TrcDbgTrace;
import trclib.TrcVisionTask;

public abstract class FrcOpenCVDetector implements TrcVisionTask.VisionProcessor<Mat, MatOfRect>
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
    private MatOfRect detectedObjects;
    private TrcVisionTask<Mat, MatOfRect> visionTask;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FrcOpenCVDetector(String instanceName)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        videoIn = CameraServer.getInstance().getVideo();
        videoOut = CameraServer.getInstance().putVideo("Rectangle", 640, 480);

        image = new Mat();
        detectedObjects = new MatOfRect();
        visionTask = new TrcVisionTask<>(this, image, detectedObjects);
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
        return videoIn.grabFrame(image) != 0;
    }   //grabFrame

    /**
     * This method is called to render an image to the video output and overlay detected objects on top of it.
     * 
     * @param image specifies the frame to be rendered to the video output.
     * @param detectedObjects specifies the detected objects.
     */
    @Override
    public void putFrame(Mat image, MatOfRect detectedObjects)
    {
        //
        // Overlay a rectangle on each detected object.
        //
        Rect[] rects = detectedObjects.toArray();
        for (Rect r: rects)
        {
            //
            // Draw a rectangle around the detected object.
            //
            Imgproc.rectangle(
                image, new Point(r.x, r.y), new Point(r.x + r.width, r.y + r.height), new Scalar(0, 255, 0));
        }

        videoOut.putFrame(image);
    }   //putFrame

}   //class FrcOpenCVDetector
