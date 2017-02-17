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

class CmdMidGearLift implements TrcRobot.RobotCommand
{
    private enum State
    {
        DO_DELAY,
        MOVE_FORWARD1,
        VISION_GEAR_DEPLOY,
        MOVE_SIDEWAYS,
        MOVE_FORWARD2, 
        DONE
    }   //enum State

    private static final String moduleName = "CmdMidGearLift";

    private Robot robot;
    private double delay;
    private double forward1Distance;
    private double forward2Distance;
    private double sidewaysDistance;
    private double heading;
    private CmdVisionGearDeploy cmdVisionDeploy;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;

    CmdMidGearLift(Robot robot, double delay, double forward1Distance, double forward2Distance, double sidewaysDistance)
    {
        this.robot = robot;
        this.delay = delay;
        this.forward1Distance = forward1Distance;
        this.forward2Distance = forward2Distance;
        this.sidewaysDistance = sidewaysDistance;
        this.heading = 0.0;

        //MTS: Need to change to final parameters.
        // cmdVisionDeploy = new CmdVisionGearDeploy(
        //    robot, sidewaysDistance, sidewaysDistance, sidewaysDistance, sidewaysDistance);
        // TODO add proper instantiation of cmdVisionDeploy
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DO_DELAY);
    }   //CmdMidGearLift

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
            boolean printStateInfo = true;
            state = sm.getState();

            switch (state)
            {
                case DO_DELAY:
                    //
                    // Do delay if any.
                    //
                    if (delay == 0.0)
                    {
                        sm.setState(State.MOVE_FORWARD1);
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.MOVE_FORWARD1);
                    }
                    break;

                case MOVE_FORWARD1:
                    //
                    // Drive the set distance and heading.
                    //
                    robot.pidDrive.setTarget(0.0, forward1Distance, heading, false, event);
                    sm.waitForSingleEvent(event, State.VISION_GEAR_DEPLOY);
                    break;

                case VISION_GEAR_DEPLOY:
                    printStateInfo = false;
                    if (cmdVisionDeploy.cmdPeriodic(elapsedTime))
                    {
                        sm.setState(State.MOVE_SIDEWAYS);
                    }
                    break;

                case MOVE_SIDEWAYS:
                    robot.pidDrive.setTarget(sidewaysDistance, 0.0, heading, false, event);
                    sm.waitForSingleEvent(event, State.MOVE_FORWARD2);
                    break;

                case MOVE_FORWARD2:
                    robot.pidDrive.setTarget(0.0, forward2Distance, heading, false, event);
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

            if (printStateInfo)
            {
                robot.traceStateInfo(elapsedTime, state.toString());
            }
        }

        return done;
    }   //cmdPeriodic

}   //class CmdMidGearLift
