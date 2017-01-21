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
import org.opencv.core.Rect;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import frclib.FrcFaceDetector;
import frclib.FrcVisionTask;

public class FaceDetection implements FrcVisionTask.VideoDevice
{
    private static final String CLASSIFIER_PATH = "cascade-files/haarcascade_frontalface_alt.xml";
    private CvSink videoIn;
    private CvSource videoOut;
    private FrcVisionTask visionTask;

    public FaceDetection()
    {
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(640, 480);

        videoIn = CameraServer.getInstance().getVideo();
        videoOut = CameraServer.getInstance().putVideo("Rectangle", 640, 480);
        visionTask = new FrcVisionTask(this, new FrcFaceDetector("FrontalFace", CLASSIFIER_PATH));
    }   //FaceDetection

    public void setEnabled(boolean enabled)
    {
        visionTask.setTaskEnabled(enabled);
    }   //setEnabled

    public Rect[] getFaceRects()
    {
        Rect[] faceRects = visionTask.getObjectRects();

        return faceRects;
    }   //getFaceRects

    //
    // Implements TrcVisionTask.VideoDevice interface.
    //

    /**
     * This method is called to grab a frame from the video input.
     *
     * @param frame specifies the frame buffer to hold the captured image.
     * @return true if frame is successfully captured, false otherwise.
     */
    @Override
    public boolean grabFrame(Mat frame)
    {
        return videoIn.grabFrame(frame) != 0;
    }   //grabFrame

    /**
     * This method is called to render a frame to the video output.
     * 
     * @param frame specifies the frame to be rendered to the video output.
     */
    public void putFrame(Mat frame)
    {
        videoOut.putFrame(frame);
    }   //putFrame

}   //class VisionTarget
