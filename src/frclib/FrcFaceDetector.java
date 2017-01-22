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

import org.opencv.core.Mat;
import org.opencv.core.MatOfRect;
import org.opencv.core.Rect;
import org.opencv.objdetect.CascadeClassifier;

import trclib.TrcDbgTrace;

public class FrcFaceDetector extends FrcOpenCVDetector
{
    private static final String moduleName = "FrcFaceDetector";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private CascadeClassifier faceDetector;
    private Rect[] faceRects = null;

    /**
     * Constructor: Create an instance of the object.
     */
    public FrcFaceDetector(final String instanceName, final String classifierPath)
    {
        super(instanceName);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        faceDetector = new CascadeClassifier(classifierPath);
        if (faceDetector.empty())
        {
            throw new RuntimeException("Failed to load Cascade Classifier <" + classifierPath + ">");
        }
    }   //FrcFaceDetector

    /**
     * This method returns an array of rectangles of last detected faces.
     *
     * @return array of rectangle of last detected faces.
     */
    public Rect[] getFaceRects()
    {
        final String funcName = "getFaceRects";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        return faceRects;
    }   //getFaceRects

    //
    // Implements the FrcVisionTask.VisionProcesor interface.
    //

    /**
     * This method is called to detect objects in the image frame.
     *
     * @param image specifies the image to be processed.
     * @param detectedObjects specifies the object rectangle array to hold the detected objects.
     * @return true if detected objects, false otherwise.
     */
    @Override
    public boolean detectObjects(Mat image, MatOfRect detectedObjects)
    {
        final String funcName = "detectedObjects";
        boolean found = false;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.CALLBK, "image=%s,objRects=%s",
                image.toString(), detectedObjects.toString());
        }

        faceDetector.detectMultiScale(image, detectedObjects);
        if (!detectedObjects.empty())
        {
            faceRects = detectedObjects.toArray();
            found = true;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.CALLBK, "=%s", Boolean.toString(found));
        }

        return found;
    }   //detectedObjects

}   //class FrcFaceDetector
