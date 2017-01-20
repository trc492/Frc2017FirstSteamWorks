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

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

//import java.awt.Image;
//import java.util.Comparator;
//import java.util.Vector;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import hallib.HalDashboard;
import trclib.TrcDbgTrace;
import trclib.TrcUtil;

public class FrcVision implements Runnable
{
    private static final String moduleName = "FrcVision";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private static final boolean visionPerfEnabled = false;

//    public class ParticleReport implements Comparator<ParticleReport>, Comparable<ParticleReport>
//    {
//        public int imageWidth;
//        public int imageHeight;
//        public double percentAreaToImageArea;
//        public double area;
//        public double boundingRectLeft;
//        public double boundingRectTop;
//        public double boundingRectRight;
//        public double boundingRectBottom;
//
//        public int compareTo(ParticleReport r)
//        {
//            return (int)(r.area - this.area);
//        }
//
//        public int compare(ParticleReport r1, ParticleReport r2)
//        {
//            return (int)(r1.area - r2.area);
//        }
//    }   //class ParticleReport

    private CvSink videoIn;
    private CvSource videoOut;
    private Mat mat;
//    private ColorMode colorMode;
//    private Range[] colorThresholds;
//    private boolean doConvexHull;
//    private ParticleFilterCriteria2[] filterCriteria;
//    private ParticleFilterOptions2 filterOptions;

//    private Image binaryImage;
    private Object monitor;
    private Thread visionThread = null;

    private long processingInterval = 50;   // in msec
    private boolean taskEnabled = false;
    private boolean oneShotEnabled = false;
//    private Vector<ParticleReport> targets = null;

    public FrcVision(CvSink videoIn, CvSource videoOut)
//            ImageType imageType,
//            ColorMode colorMode,
//            Range[] colorThresholds,
//            boolean doConvexHull,
//            ParticleFilterCriteria2[] filterCriteria,
//            ParticleFilterOptions2 filterOptions)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        this.videoIn = videoIn;
        this.videoOut = videoOut;
        mat = new Mat();
//        this.colorMode = colorMode;
//        this.colorThresholds = colorThresholds;
//        this.doConvexHull = doConvexHull;
//        this.filterCriteria = filterCriteria;
//        this.filterOptions = filterOptions;
//        if (colorThresholds.length != 3)
//        {
//            throw new IllegalArgumentException(
//                    "Color threshold array must have 3 elements.");
//        }
//
//        binaryImage = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);

        monitor = new Object();
        visionThread = new Thread(this, "VisionTask");
        visionThread.setDaemon(true);
        visionThread.start();
    }   //FrcVision

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

//    public Vector<ParticleReport> getTargets()
//    {
//        final String funcName = "getTargets";
//        if (debugEnabled)
//        {
//            dbgTrace.traceEnter(
//                    funcName, TrcDbgTrace.TraceLevel.API);
//        }
//
//        Vector<ParticleReport> newTargets = null;
//        synchronized(monitor)
//        {
//            if (!taskEnabled && targets == null)
//            {
//                oneShotEnabled = true;
//                monitor.notify();
//            }
//            newTargets = targets;
//            targets = null;
//        }
//
//        if (debugEnabled)
//        {
//            dbgTrace.traceExit(
//                    funcName, TrcDbgTrace.TraceLevel.API);
//        }
//        return newTargets;
//    }   //getTargets

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

    private void processImage()
    {
        final String funcName = "processImage";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK);
        }

        double totalTime = 0.0;
        double startTime;
        double deltaTime;
        //
        // Grab a frame from the camera and put it in the source mat. If there is an error notify the output.
        //
        if (videoIn.grabFrame(mat) == 0)
        {
            //
            // Send the error to the output.
            //
            videoOut.notifyError(videoIn.getError());
        }
        else
        {
            //
            // Put a rectangle on the image.
            //
            Imgproc.rectangle(mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
            // Give the output stream a new image to display
            videoOut.putFrame(mat);
        }

//        if (image != null)
//        {
//            if (visionPerfEnabled)
//            {
//                startTime = TrcUtil.getCurrentTime();
//            }
//            NIVision.imaqColorThreshold(
//                    binaryImage,
//                    image,
//                    255,
//                    colorMode,
//                    colorThresholds[0],
//                    colorThresholds[1],
//                    colorThresholds[2]);
//            if (visionPerfEnabled)
//            {
//                deltaTime = TrcUtil.getCurrentTime() - startTime;
//                totalTime += deltaTime;
//                HalDashboard.putNumber("ColorThresholdTime", deltaTime);
//            }
//
//            if (doConvexHull)
//            {
//                if (visionPerfEnabled)
//                {
//                    startTime = TrcUtil.getCurrentTime();
//                }
//                NIVision.imaqConvexHull(binaryImage, binaryImage, 1);
//                if (visionPerfEnabled)
//                {
//                    deltaTime = TrcUtil.getCurrentTime() - startTime;
//                    totalTime += deltaTime;
//                    HalDashboard.putNumber("ConvexHullTime", deltaTime);
//                }
//            }
//
//            if (visionPerfEnabled)
//            {
//                startTime = TrcUtil.getCurrentTime();
//            }
//            NIVision.imaqParticleFilter4(
//                    binaryImage,
//                    binaryImage,
//                    filterCriteria,
//                    filterOptions,
//                    null);
            if (visionPerfEnabled)
            {
                deltaTime = TrcUtil.getCurrentTime() - startTime;
                totalTime += deltaTime;
                HalDashboard.putNumber("ParticleFilterTime", deltaTime);
            }

//            int numParticles = NIVision.imaqCountParticles(binaryImage, 1);
//            if(numParticles > 0)
//            {
//                //
//                // Measure particles and sort by particle size.
//                //
//                Vector<ParticleReport> particles = new Vector<ParticleReport>();
//                GetImageSizeResult imageSize =
//                        NIVision.imaqGetImageSize(binaryImage);
//                if (visionPerfEnabled)
//                {
//                    startTime = HalUtil.getCurrentTime();
//                }
//
//                for(int i = 0; i < numParticles; i++)
//                {
//                    ParticleReport par = new ParticleReport();
//                    par.imageWidth = imageSize.width;
//                    par.imageHeight = imageSize.height;
//                    par.percentAreaToImageArea =
//                            NIVision.imaqMeasureParticle(
//                                    binaryImage,
//                                    i,
//                                    0,
//                                    MeasurementType.MT_AREA_BY_IMAGE_AREA);
//                    par.area =
//                            NIVision.imaqMeasureParticle(
//                                    binaryImage,
//                                    i,
//                                    0,
//                                    MeasurementType.MT_AREA);
//                    par.boundingRectTop =
//                            NIVision.imaqMeasureParticle(
//                                    binaryImage,
//                                    i,
//                                    0,
//                                    MeasurementType.MT_BOUNDING_RECT_TOP);
//                    par.boundingRectLeft =
//                            NIVision.imaqMeasureParticle(
//                                    binaryImage,
//                                    i,
//                                    0,
//                                    MeasurementType.MT_BOUNDING_RECT_LEFT);
//                    par.boundingRectBottom =
//                            NIVision.imaqMeasureParticle(
//                                    binaryImage,
//                                    i,
//                                    0,
//                                    MeasurementType.MT_BOUNDING_RECT_BOTTOM);
//                    par.boundingRectRight =
//                            NIVision.imaqMeasureParticle(
//                                    binaryImage,
//                                    i,
//                                    0,
//                                    MeasurementType.MT_BOUNDING_RECT_RIGHT);
//                    particles.add(par);
//                }
//                particles.sort(null);
//                if (visionPerfEnabled)
//                {
//                    deltaTime = HalUtil.getCurrentTime() - startTime;
//                    totalTime += deltaTime;
//                    HalDashboard.putNumber("PrepareReportTime", deltaTime);
//                }
//
//                synchronized(monitor)
//                {
//                    oneShotEnabled = false;
//                    targets = particles;
//                    particles = null;
//                }
//            }
//        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //processImage

}   //class FrcVision
