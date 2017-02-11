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

import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdVisionGearDeploy implements TrcRobot.RobotCommand
{
    private enum State
    {
        DO_DELAY,
        ALIGN_WITH_TARGET,
        DEPLOY_GEAR,
        BACKUP,
        DONE
    }   //enum State

    private static final String moduleName = "CmdPidDrive";

    private Robot robot;
    private double delay;
    private double extendTime;
    private double backupDistance;
    private double heading;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;

    CmdVisionGearDeploy(Robot robot, double delay, double extendTime, double backupDistance, double heading)
    {
        this.robot = robot;
        this.delay = delay;
        this.extendTime = extendTime;
        this.backupDistance = backupDistance;
        this.heading = heading;
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DO_DELAY);
    }   //CmdPidDrive

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
                case DO_DELAY:
                    //
                    // Do delay if any.
                    //
                    if (delay == 0.0)
                    {
                        sm.setState(State.ALIGN_WITH_TARGET);
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.ALIGN_WITH_TARGET);
                    }
                    break;
                    
                case ALIGN_WITH_TARGET:
                    //
                    // Position robot at target.
                	//
                	boolean aligned = false;
                	while(!aligned)
            		{
	                	double pos = robot.pixyVision.getTargetPosition();
            		}
                	
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

}   //class CmdPidDrive
