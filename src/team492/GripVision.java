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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import frclib.FrcRobotBase;
import frclib.FrcVisionTarget;
import trclib.TrcDbgTrace;

public class GripVision extends FrcVisionTarget
{
    private Relay ringLightPower;
    private GripPipeline pipeline;
    private TrcDbgTrace tracer;
    private Rect lastTargetRect = null;

    public GripVision(final String instanceName, CvSink videoIn, CvSource videoOut)
    {
        super(instanceName, videoIn, videoOut);

        ringLightPower = new Relay(RobotInfo.RELAY_RINGLIGHT_POWER);
        ringLightPower.setDirection(Direction.kForward);
        pipeline = new GripPipeline();
        tracer = FrcRobotBase.getGlobalTracer();
    }   //GripVision

    public void setEnabled(boolean enabled)
    {
        ringLightPower.set(enabled? Value.kOn: Value.kOff);
        super.setEnabled(enabled);
    }   //setEnabled

    public Rect getTargetRect()
    {
        if (isEnabled())
        {
            Rect[] objectRects = getObjectRects();

            tracer.traceInfo("VisionTarget", "%d object(s) found", objectRects != null? objectRects.length: 0);
            if (objectRects != null && objectRects.length >= 2)
            {
                for (int i = 0; i < objectRects.length; i++)
                {
                    tracer.traceInfo("VisionTarget", "%02d: x=%d, y=%d, width=%d, height=%d",
                        i, objectRects[i].x, objectRects[i].y, objectRects[i].width, objectRects[i].height);
                }
                //
                // Sort the detected objects by area from largest to smallest.
                //
                Arrays.sort(
                    objectRects,
                    new Comparator<Rect>()
                    {
                        public int compare(Rect rect1, Rect rect2)
                        {
                            return rect2.width*rect2.height - rect1.width*rect1.height;
                        }
                    });

                int targetRectX1 = Math.min(objectRects[0].x, objectRects[1].x);
                int targetRectY1 = Math.min(objectRects[0].y, objectRects[1].y);
                int targetRectX2 = Math.max(objectRects[0].x + objectRects[0].width,
                                            objectRects[1].x + objectRects[1].width);
                int targetRectY2 = Math.max(objectRects[0].y + objectRects[0].height,
                                            objectRects[1].y + objectRects[1].height);
                int targetRectWidth = targetRectX2 - targetRectX1;
                int targetRectHeight = targetRectY2 - targetRectY1;

                lastTargetRect = new Rect(targetRectX1, targetRectY1, targetRectWidth, targetRectHeight);
                tracer.traceInfo("PixyVision", "TargetRect: x=%d, y=%d, w=%d, h=%d",
                    lastTargetRect.x, lastTargetRect.y, lastTargetRect.width, lastTargetRect.height);
            }
        }

        return lastTargetRect;
    }   //getTargetRect

    //
    // Implements FrcVisionTarget abstract methods.
    //

    @Override
    public void processImage(Mat image)
    {
        pipeline.process(image);
    }   //processImage

    @Override
    public ArrayList<MatOfPoint> getDetectedObjects()
    {
        return pipeline.findContoursOutput();
    }   //getDetectedObjects

}   //class GripVision
