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

import frclib.FrcRobotBase;
import hallib.HalDashboard;
import trclib.TrcDbgTrace;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;

class CmdVisionGearDeploy implements TrcRobot.RobotCommand
{
    private static enum State
    {
        ALIGN_WITH_PEG, DRIVE_TO_TARGET, DEPLOY_GEAR, BACKUP, DONE
    }   //enum State

    private static final String moduleName = "CmdVisionGearDeploy";

    private TrcDbgTrace tracer = FrcRobotBase.getGlobalTracer();
    private Robot robot;
    private double idealTargetDistance;
    private double distanceToTarget;
    private double extendTime;
    private double backupDistance;
    private double turnAlignmentThreshold;
    private double distanceAlignmentThreshold;

    private TrcEvent event;
    private TrcStateMachine<State> sm;
    private PixyVision.TargetInfo lastTargetInfo = null;

    CmdVisionGearDeploy(Robot robot)
    {
        this.robot = robot;

        idealTargetDistance  = Math.abs(HalDashboard.getNumber("IdealTargetDistance", 6.0));
        distanceToTarget = HalDashboard.getNumber("DistanceToTarget", 12.0);
        extendTime = Math.abs(HalDashboard.getNumber("PneumaticExtendTime", 0.3));
        backupDistance = Math.abs(HalDashboard.getNumber("BackupDistance", 36.0));
        turnAlignmentThreshold = Math.abs(HalDashboard.getNumber("TurnAlignmentThreshold", 10.0));
        distanceAlignmentThreshold = Math.abs(HalDashboard.getNumber("DistanceAlignmentThreshold", 10.0));

        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DRIVE_TO_TARGET);//ALIGN_HORIZONTALLY);
    }   //CmdVisionGearDeploy

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean done = !sm.isEnabled();
        //
        // Print debug info.
        //
        State state = sm.getState();
        robot.dashboard.displayPrintf(1, "State: %s", state != null ? state.toString() : "Disabled");

        if (sm.isReady())
        {
            double xDistance, yDistance;
            PixyVision.TargetInfo targetInfo = null;
            state = sm.getState();

            switch (state)
            {
                case ALIGN_WITH_PEG:
                    targetInfo = robot.frontPixy.getTargetInfo();
                    tracer.traceInfo("GearDeploy", "Target Info: %s",
                        targetInfo != null? targetInfo.toString(): "not found");
                    if(targetInfo == null)
                    {
                        //
                        // Can't find the target for some reason, let's just do it blind.
                        //
                        sm.setState(State.DRIVE_TO_TARGET);
                    }
                    else if (Math.abs(targetInfo.angle) <= turnAlignmentThreshold &&
                             Math.abs(targetInfo.distance - idealTargetDistance) <= distanceAlignmentThreshold)
                    {
                        //
                        // We are there, go for it.
                        //
                        sm.setState(State.DEPLOY_GEAR);
                    }
                    else
                    {
                        //
                        // See the target but not quite there yet, let's move there.
                        //
                        xDistance = 0.0;
                        yDistance = targetInfo.distance - idealTargetDistance;
                        if (lastTargetInfo == null)
                        {
                            //
                            // If lastTargetInfo is null, this is the first half run, so half the distance.
                            //
                            yDistance /= 2.0;
                            lastTargetInfo = targetInfo;
                        }
                        robot.targetHeading += targetInfo.angle;

                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                        sm.waitForSingleEvent(event, State.ALIGN_WITH_PEG);
//                        lastDistanceToTarget = (idealTargetSize - actualTargetSize)*inchesPerPixel;
//                        double midpoint = getHorizontalPosition();
//                        double screenMidpoint = RobotInfo.CAM_WIDTH/2;
//                        double turnErrorMargin = midpoint - screenMidpoint;
//                        double angleToTarget = Math.toDegrees(Math.atan((turnErrorMargin*inchesPerPixel)/lastDistanceToTarget));
//                        robot.targetHeading = angleToTarget + robot.driveBase.getHeading();
//                        if(Math.abs(turnErrorMargin) > turnAlignmentThreshold || Math.abs(lastDistanceToTarget) > distanceAlignmentThreshold)
//                        {
//                            robot.pidDrive.setTarget(0.0, lastDistanceToTarget/2, robot.targetHeading, false, event);
//                            sm.waitForSingleEvent(event, State.ALIGN_WITH_PEG_2);
//                        }
//                        else
//                        {
//                            sm.setState(State.DEPLOY_GEAR);
//                        }
                    }
                    break;
                    
//                case ALIGN_WITH_PEG_2:
//                	//
//                    // Position robot at target again.
//                    // get info from vision (m: alignment, n:distance)
//                	double newTargetSize = getTargetSize();
//                	if(newTargetSize == -1)
//                	{
//                		robot.pidDrive.setTarget(0.0, lastDistanceToTarget/2, robot.targetHeading, false, event);
//                        sm.waitForSingleEvent(event, State.DEPLOY_GEAR);
//                	}
//                	else
//                	{
//                	    double newDistanceToTarget = (idealTargetSize - newTargetSize)*inchesPerPixel;
//                        double midpoint = getHorizontalPosition();
//                        double screenMidpoint = RobotInfo.CAM_WIDTH/2;
//                        double turnErrorMargin = midpoint - screenMidpoint;
//                        double angleToTarget = Math.atan((turnErrorMargin*inchesPerPixel)/newDistanceToTarget);
//                        robot.targetHeading = angleToTarget + robot.driveBase.getHeading();
//                        if(Math.abs(turnErrorMargin) > turnAlignmentThreshold || Math.abs(newDistanceToTarget) > distanceAlignmentThreshold)
//                        {
//                            robot.pidDrive.setTarget(0.0, newDistanceToTarget, robot.targetHeading, false, event);
//                            sm.waitForSingleEvent(event, State.DEPLOY_GEAR);
//                        }
//                        else
//                        {
//                    	    sm.setState(State.DEPLOY_GEAR);
//                        }
//                	}
//                    break;

                case DRIVE_TO_TARGET:
                    xDistance = 0.0;
                    if (lastTargetInfo == null)
                    {
                        //
                        // Going there blind.
                        //
//                        yDistance = distanceToTarget;
                        yDistance = robot.getUltrasonicDistance() - 12.0;
                    }
                    else
                    {
                        //
                        // Going the other half of the distance from last time.
                        //
                        yDistance = (lastTargetInfo.distance - idealTargetDistance)/2.0;
                    }
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DEPLOY_GEAR);
//                	double actualTargetSize = getTargetSize();
//                	double distanceErrorMargin = idealTargetSize - actualTargetSize;
//                	if ( distanceErrorMargin > distanceAlignmentThreshold)
//                	{
//                		double distanceTarget = distanceErrorMargin*distanceAlignmentMultiplier;
//                	    robot.pidDrive.setTarget(0.0, distanceTarget, robot.targetHeading, false, event);
//                        sm.waitForSingleEvent(event, State.ALIGN_DISTANCE);
//                	}
//                	else
//                	{
//                		sm.setState(State.DEPLOY_GEAR);
//                	}
                    break;
                    
                case DEPLOY_GEAR:
                    //
                    // Place gear on axle.
                    //
                    robot.mailbox.extend(extendTime, event);
                    sm.waitForSingleEvent(event, State.BACKUP);
                    break;

                case BACKUP:
                    //
                    // Backup from axle.
                    //
                    xDistance = 0.0;
                    yDistance = -backupDistance;
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    robot.mailbox.retract();
                    done = true;
                    sm.stop();
                    break;
            }
            robot.traceStateInfo(elapsedTime, state.toString());
        }

        return done;
    }   //cmdPeriodic

//    private double getHorizontalPosition()
//    {
//    	Rect memeTangle = robot.frontPixy.getTargetRect();
//        double midpoint = memeTangle.x + memeTangle.width / 2;
//        return midpoint;
//    }
//    private double getTargetSize()
//    {
//        Rect dankTangle = robot.frontPixy.getTargetRect();
//        if(dankTangle == null)
//        {
//        	double targetSize = -1;
//        	return targetSize;
//        }
//        else
//        {
//        double targetSize = dankTangle.height;
//        return targetSize;
//        }
//    }
    
}   //class CmdVisionGearDeploy
