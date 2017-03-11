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
        TURN_TO_TARGET,
        DRIVE_TOWARDS_TARGET,
        ALIGN_WITH_TARGET,
        DRIVE_TO_TARGET,
        DEPLOY_GEAR,
        BACKUP,
        DONE
    }   //enum State

    private static final String moduleName = "CmdVisionGearDeploy";

    private TrcDbgTrace tracer = FrcRobotBase.getGlobalTracer();
    private Robot robot;

    private double visionFarDistance;
    private double visionNearDistance;
    private double visionGearDeployTime;
    private double visionBackupDistance;

    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;

    CmdVisionGearDeploy(Robot robot)
    {
        this.robot = robot;

        visionFarDistance = HalDashboard.getNumber("VisionFarDistance", 24.0);
        visionNearDistance = HalDashboard.getNumber("VisionNearDistance", 12.0);
        visionGearDeployTime = HalDashboard.getNumber("VisionGearDeployTime", 0.3);
        visionBackupDistance = HalDashboard.getNumber("VisionBackupDistance", 36.0);

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.TURN_TO_TARGET);

        robot.tracer.traceInfo(
            moduleName, "farDist=%.1f, nearDist=%.1f, deployTime=%.1f, backupDist=%.1f",
            visionFarDistance, visionNearDistance, visionGearDeployTime, visionBackupDistance);
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
        State nextState;
        robot.dashboard.displayPrintf(1, "State: %s", state != null ? state.toString() : "Disabled");

        if (sm.isReady())
        {
            double xDistance, yDistance;
            PixyVision.TargetInfo targetInfo = null;
            state = sm.getState();

            switch (state)
            {
                case TURN_TO_TARGET:
                    targetInfo = robot.frontPixy.getTargetInfo();
                    double angle = targetInfo != null? targetInfo.angle: 0.0;
                    xDistance = yDistance = 0.0;
                    robot.targetHeading += angle;
                    nextState = Math.abs(angle) <= RobotInfo.GYRO_TURN_TOLERANCE?
                        State.DRIVE_TOWARDS_TARGET: State.TURN_TO_TARGET;

                    tracer.traceInfo(moduleName, "Target Info: %s",
                        targetInfo != null? targetInfo.toString(): "not found");

                    robot.setPidDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case DRIVE_TOWARDS_TARGET:
                    xDistance = 0.0;
                    yDistance = robot.getUltrasonicDistance() - visionFarDistance;

                    robot.setPidDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.ALIGN_WITH_TARGET);
                    break;

                case ALIGN_WITH_TARGET:
                    targetInfo = robot.frontPixy.getTargetInfo();
                    xDistance = targetInfo != null? targetInfo.xDistance: 0.0;
                    yDistance = 0.0;
                    nextState = Math.abs(xDistance) <= RobotInfo.ENCODER_X_TOLERANCE?
                        State.DRIVE_TO_TARGET: State.ALIGN_WITH_TARGET;

                    tracer.traceInfo(moduleName, "Target Info: %s",
                        targetInfo != null? targetInfo.toString(): "not found");

                    robot.setPidDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case DRIVE_TO_TARGET:
                    xDistance = 0.0;
                    yDistance = robot.getUltrasonicDistance() - visionNearDistance;

                    robot.setPidDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
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
                    robot.setPidDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
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
