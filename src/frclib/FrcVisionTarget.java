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

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import team492.GripPipeline;

/**
 * This class implements a Vision Targeting system that uses a GRIP pipeline and OpenCV. It uses a separate vision
 * thread and will take care of thread synchronization so making the use of this class extremely simple.
 */
public class FrcVisionTarget extends FrcOpenCVDetector<ArrayList<MatOfPoint>>
{
    private GripPipeline pipeline;
    private volatile Mat image = null;
    private volatile Rect[] targetRects = null;
    private boolean videoOutEnabled = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param videoIn specifies the video input stream.
     * @param videoOut specifies the video output stream.
     */
    public FrcVisionTarget(final String instanceName, CvSink videoIn, CvSource videoOut, GripPipeline pipeline)
    {
        super(instanceName, videoIn, videoOut);
        this.pipeline = pipeline;
    }   //FrcVisionTarget

    /**
     * This method returns an array of rectangles of last detected targets.
     *
     * @return array of rectangle of last detected targets.
     */
    public Rect[] getTargetRects()
    {
        return targetRects;
    }   //getObjectRects

    /**
     * This method update the video stream with the detected targets overlay on the image as rectangles.
     *
     * @param color specifies the color of the rectangle outline overlay onto the detected targets.
     * @param thickness specifies the thickness of the rectangle outline.
     */
    public void putFrame(Scalar color, int thickness)
    {
        if (image != null)
        {
            super.putFrame(image, targetRects, color, thickness);
        }
    }   //putFrame

    /**
     * This method update the video stream with the detected targets overlay on the image as rectangles.
     */
    public void putFrame()
    {
        if (image != null)
        {
            super.putFrame(image, targetRects, new Scalar(0, 255, 0), 0);
        }
    }   //putFrame

    /**
     * This method enables/disables the video out stream.
     *
     * @param enabled specifies true to enable video out stream, false to disable.
     */
    public void setVideoOutEnabled(boolean enabled)
    {
        videoOutEnabled = enabled;
    }   //setVideoOutEnabled

    //
    // Implements the TrcVisionTask.VisionProcessor.detectObjects method.
    //

    /**
     * This method is called to detect objects in the acquired image frame.
     *
     * @param image specifies the image to be processed.
     * @param detectedTargets specifies the preallocated buffer to hold the detected targets (not used since no
     *        preallocated buffer required).
     * @return detected objects, null if none detected.
     */
    @Override
    public ArrayList<MatOfPoint> detectObjects(Mat image, ArrayList<MatOfPoint> detectedObjects)
    {
        ArrayList<MatOfPoint> detectedTargets = null;
        //
        // Process the image to detect the objects we are looking for and put them into detectedObjects.
        //
        pipeline.process(image);
        detectedTargets = pipeline.filterContoursOutput();
        //
        // If we detected any objects, convert them into an array of rectangles.
        //
        if (!detectedTargets.isEmpty())
        {
            targetRects = new Rect[detectedTargets.size()];
            for (int i = 0; i < targetRects.length; i++)
            {
                targetRects[i] = Imgproc.boundingRect(detectedTargets.get(i));
            }
        }
        else
        {
            targetRects = null;
            detectedTargets = null;
        }

        if (videoOutEnabled)
        {
            putFrame();
        }

        return detectedTargets;
    }   //detectObjects

}   //class FrcVisionTarget
