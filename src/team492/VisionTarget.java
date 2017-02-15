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

package team492;

import org.opencv.core.Mat;
import org.opencv.core.MatOfRect;
import org.opencv.core.Rect;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import frclib.FrcOpenCVDetector;

public class VisionTarget extends FrcOpenCVDetector
{
    private CvSource videoOut;
    private Rect[] objRects = null;
    private boolean videoOutEnabled = false;

    public VisionTarget(final String instanceName, CvSink videoIn, CvSource videoOut)
    {
        super(instanceName, videoIn, videoOut);
        this.videoOut = videoOut;
    }

    public Rect[] getObjectRects()
    {
        return objRects;
    }   //getObjectRects

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
     * @param detectedObjects specifies the object rectangle array to hold the detected objects.
     * @return true if detected objects, false otherwise.
     */
    @Override
    public boolean detectObjects(Mat image, MatOfRect detectedObjects)
    {
        boolean found = false;
        //
        // Process the image to detect the objects we are looking for and put them into detectedObjects.
        //

        //
        // If we detected any objects, convert them into an array of rectangles.
        //
        if (!detectedObjects.empty())
        {
            objRects = detectedObjects.toArray();
            found = true;
        }

        if (videoOutEnabled)
        {
            videoOut.putFrame(image);
        }

        return found;
    }   //detectObjects

}   //class VisionTarget
