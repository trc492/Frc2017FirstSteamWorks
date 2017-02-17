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

import org.opencv.core.Rect;

import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdVisionGearDeploy implements TrcRobot.RobotCommand
{
    private enum State
    {
        ALIGN_HORIZONTALLY,
        ALIGN_DISTANCE,
        DEPLOY_GEAR,
        BACKUP,
        DONE
    }   //enum State

    private static final String moduleName = "CmdVisionGearDeploy";

    private Robot robot;
    private double extendTime;
    private double backupDistance;
    private double horizontalAlignmentThreshold;
    private double distanceAlignmentThreshold;
    private double heading;
    private double alignmentSensitivity;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;

    CmdVisionGearDeploy(Robot robot, double extendTime, double backupDistance, 
    		double horizontalAlignmentThreshold, double distanceAlignmentThreshold,
    		double alignmentSensitivity, double heading)
    {
        this.robot = robot;
        this.extendTime = extendTime;
        this.backupDistance = backupDistance;
        this.heading = heading;
        this.horizontalAlignmentThreshold = horizontalAlignmentThreshold;
        this.distanceAlignmentThreshold = distanceAlignmentThreshold;
        this.alignmentSensitivity = alignmentSensitivity;
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.ALIGN_HORIZONTALLY);
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
        robot.dashboard.displayPrintf(1, "State: %s", state != null? sm.getState().toString(): "Disabled");

        if (sm.isReady())
        {
            state = sm.getState();

            switch (state)
            {
            	
                case ALIGN_HORIZONTALLY:
                    //
                    // Position robot at target.
                	// get info from vision (m: alignment, n:distance)
                	double midpoint = getHorizontalPosition();
                	double screenMidpoint = 160;
                	// TODO find screenMidpoint
                	double errorMargin = Math.abs(midpoint - screenMidpoint);
                	if(errorMargin > horizontalAlignmentThreshold)
                	{ 
                		// is aligned
                		double strafeSpeed = Math.signum(errorMargin);
                        robot.pidDrive.setTarget(0.0, backupDistance, 0.0, false, event);
                	}
                    sm.waitForSingleEvent(event, State.ALIGN_HORIZONTALLY);
                    break;
                case ALIGN_DISTANCE:
                    sm.waitForSingleEvent(event, State.DEPLOY_GEAR);
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
                    robot.pidDrive.setTarget(0.0, backupDistance, 0.0, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    done = true;
                    sm.stop();
                    break;
            }
            robot.traceStateInfo(elapsedTime, state.toString());
        }

        return done;
    }   //cmdPeriodic

    private double getHorizontalPosition()
    {
    	Rect r = robot.visionTarget.getTargetRect();
    	double midpoint = r.x + r.width / 2;
    	return midpoint;
    }
    
}   //class CmdPidDrive
