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

import java.util.Arrays;
import java.util.List;

//MTS: import team492.CmdMidGearLift.State;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdSideGearLift implements TrcRobot.RobotCommand
{
    private enum State
    {
        DO_DELAY,
        FORWARD1,
        TURN_TO_AIRSHIP,
        FORWARD2,
        VISION_DEPLOY,
        BACK_UP,
        TURN_FORWARD,
        GO_TO_NEUTRAL,
        DONE
    }   //enum State

    private static final String moduleName = "CmdPidDrive";

    private Robot robot;
    private double delay;
    private double distanceToBaseline;
    private double angleToAirship;
    private double baselineToAirship;
    private double baselineToNeutral;
    private CmdVisionGearDeploy visionDeploy;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;
    private List<State> values;

    CmdSideGearLift(final Robot robot,final double delay, final double distanceToBaseline, final double angleToAirship, final double baselineToAirship,final double baselineToNeutral)
    {
    	values = Arrays.asList(State.values());
        this.robot = robot;
        this.delay = delay;
        this.distanceToBaseline = distanceToBaseline;
        this.angleToAirship = angleToAirship;
        this.baselineToAirship = baselineToAirship;
        this.baselineToNeutral = baselineToNeutral;
        //MTS: Need to change to final parameters.
        visionDeploy = new CmdVisionGearDeploy(robot, baselineToNeutral, baselineToNeutral, baselineToNeutral, baselineToNeutral);
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DO_DELAY);
    }   //CmdPidDrive

    //
    // Implements the TrcRobot.RobotCommand interface.
    //
    private State getNextState(TrcStateMachine<State> sm){
    	int index = values.indexOf(sm.getState()) + 1;
    	if(index != State.values().length){
    		return State.values()[index];    		
    	}
    	else{
    		return sm.getState();
    	}
    }
    
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
                        sm.setState(getNextState(sm));
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, getNextState(sm));
                    }
                    break;

                case FORWARD1:
                    //
                    // Drive the set distance and heading.
                    //
                    robot.pidDrive.setTarget(distanceToBaseline, 0, false, event);
                    sm.waitForSingleEvent(event, getNextState(sm));
                    break;
                    
                case TURN_TO_AIRSHIP:
                	robot.pidDrive.setTarget(0, angleToAirship, false, event);
                	sm.waitForSingleEvent(event, getNextState(sm));
                	break;
                
                case FORWARD2:
                	robot.pidDrive.setTarget(baselineToAirship, 0, false, event);
                	sm.waitForSingleEvent(event, getNextState(sm));
                	break;
                
                case VISION_DEPLOY:
                	if (visionDeploy.cmdPeriodic(elapsedTime)){
                		sm.setState(getNextState(sm));
                	}
                	break;
                	
                case BACK_UP:
                	robot.pidDrive.setTarget(-baselineToAirship, 0, false, event);
                	sm.waitForSingleEvent(event, getNextState(sm));
                	break;
                	
                case TURN_FORWARD:
                	robot.pidDrive.setTarget(0, 0, false, event);
                	sm.waitForSingleEvent(event, getNextState(sm));
                	break;
                	
                case GO_TO_NEUTRAL:
                	robot.pidDrive.setTarget(baselineToNeutral, 9, false, event);
                	sm.waitForSingleEvent(event, getNextState(sm));
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
