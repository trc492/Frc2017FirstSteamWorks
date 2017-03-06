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
import trclib.TrcTimer;

class CmdVisionGearDeploy implements TrcRobot.RobotCommand
{
    private static enum State
    {
        WAIT_FOR_CAMERA, ALIGN_WITH_PEG, DRIVE_TO_TARGET_BLIND, DEPLOY_GEAR, BACKUP, DONE
    }   //enum State

    private static final String moduleName = "CmdVisionGearDeploy";

    private TrcDbgTrace tracer = FrcRobotBase.getGlobalTracer();
    private Robot robot;

    private double visionCameraSettling;
    private double visionAlignAngleTolerance;
    private double visionAlignDistanceTolerance;
    private double visionTargetDistance;
    private double visionSonarDistance;
    private double visionGearDeployTime;
    private double visionBackupDistance;

    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;
    private PixyVision.TargetInfo lastTargetInfo = null;

    CmdVisionGearDeploy(Robot robot)
    {
        this.robot = robot;

        visionCameraSettling = HalDashboard.getNumber("VisionCameraSettling", 0.2);
        visionAlignAngleTolerance = HalDashboard.getNumber("VisionAlignAngleTolerance", 1.0);
        visionAlignDistanceTolerance = HalDashboard.getNumber("VisionAlignDistanceTolerance", 1.0);
        visionTargetDistance = HalDashboard.getNumber("VisionTargetDistance", 6.0);
        visionSonarDistance = HalDashboard.getNumber("VisionSonarDistance", 12.0);
        visionGearDeployTime = HalDashboard.getNumber("VisionGearDeployTime", 0.3);
        visionBackupDistance = HalDashboard.getNumber("VisionBackupDistance", 36.0);

        event = new TrcEvent(moduleName);
        timer = new TrcTimer("VisionGear");
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.WAIT_FOR_CAMERA);
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
                case WAIT_FOR_CAMERA:
                    if (visionCameraSettling == 0.0)
                    {
                        sm.setState(State.ALIGN_WITH_PEG);
                    }
                    else
                    {
                        timer.set(visionCameraSettling, event);
                        sm.waitForSingleEvent(event, State.ALIGN_WITH_PEG);
                    }
                    break;

                case ALIGN_WITH_PEG:
                    targetInfo = robot.frontPixy.getTargetInfo();
                    tracer.traceInfo("GearDeploy", "Target Info: %s",
                        targetInfo != null? targetInfo.toString(): "not found");
                    if(targetInfo == null)
                    {
                        //
                        // Can't find the target for some reason, let's just do it blind.
                        //
                        sm.setState(State.DRIVE_TO_TARGET_BLIND);
                    }
                    else if (Math.abs(targetInfo.angle) <= visionAlignAngleTolerance &&
                             Math.abs(targetInfo.distance - visionTargetDistance) <= visionAlignDistanceTolerance)
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
                        yDistance = targetInfo.distance - visionTargetDistance;
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
                        sm.waitForSingleEvent(event, State.WAIT_FOR_CAMERA);
                    }
                    break;

                case DRIVE_TO_TARGET_BLIND:
                    xDistance = 0.0;
                    if (lastTargetInfo == null)
                    {
                        //
                        // We never see the target, so use the ultrasonic distance instead.
                        //
                        yDistance = robot.getUltrasonicDistance() - visionSonarDistance;
                    }
                    else
                    {
                        //
                        // Going the other half of the distance from last time.
                        //
                        yDistance = (lastTargetInfo.distance - visionTargetDistance)/2.0;
                    }

                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DEPLOY_GEAR);
                    break;

                case DEPLOY_GEAR:
                    //
                    // Place gear on axle.
                    //
                    robot.mailbox.extend();
                    timer.set(visionGearDeployTime, event);
                    sm.waitForSingleEvent(event, State.BACKUP);
                    break;

                case BACKUP:
                    //
                    // Backup from the lift.
                    //
                    xDistance = 0.0;
                    yDistance = -visionBackupDistance;
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

}   //class CmdVisionGearDeploy
