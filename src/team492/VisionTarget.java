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

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import frclib.FrcVisionTarget;

public class VisionTarget extends FrcVisionTarget
{
    private Relay ringLightPower;
    private GripPipeline pipeline;

    public VisionTarget(final String instanceName, CvSink videoIn, CvSource videoOut)
    {
        super(instanceName, videoIn, videoOut);

        ringLightPower = new Relay(RobotInfo.RELAY_RINGLIGHT_POWER);
        ringLightPower.setDirection(Direction.kForward);
        pipeline = new GripPipeline();
    }   //VisionTarget

    public void setEnabled(boolean enabled)
    {
        ringLightPower.set(enabled? Value.kOn: Value.kOff);
        super.setEnabled(enabled);
    }   //setEnabled

    //
    // Implements FrcVisionTarget abstract methods.
    //

    @Override
    public void processImage(Mat image)
    {
        pipeline.process(image);
    }   //processImage

    @Override
    public ArrayList<MatOfPoint> getTargets()
    {
        return pipeline.findContoursOutput();
    }   //getTargets

}   //class VisionTarget
