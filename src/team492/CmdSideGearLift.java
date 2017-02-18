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

import frclib.FrcValueMenu;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdSideGearLift implements TrcRobot.RobotCommand
{
    private static enum State
    {
        DO_DELAY,
        MOVE_FORWARD_ON_SIDE,
        TURN_TOWARDS_AIRSHIP,
        MOVE_TOWARDS_AIRSHIP,
        VISION_DEPLOY,
        BACKUP_FROM_AIRSHIP,
        TURN_TOWARDS_LOADING_STATION,
        MOVE_TOWARDS_LOADING_STATION,
        DONE
    }   //enum State

    private static final String moduleName = "CmdSideGearLift";

    private Robot robot;
    private double delay;
    private boolean rightSide;
    private double sideMoveDistance;
    private double angleToAirship;
    private double orientedAirshipMoveDistance;
    private double sidewaysDistance;
    private double loadingStationTurnAngle;
    private double loadingStationMoveDistance;
    
    private CmdVisionGearDeploy visionDeploy;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;

    CmdSideGearLift(Robot robot, double delay, boolean rightSide)
    {
        this.robot = robot;
        this.delay = delay;
        this.rightSide = rightSide;
        
        //
        // All distances from the menus are in the unit of feet.
        //
        FrcValueMenu sideMoveDistanceMenu = new FrcValueMenu("MoveDistanceOnSide", 7.0 - RobotInfo.ROBOT_LENGTH/12.0);
        FrcValueMenu angleToAirshipMenu = new FrcValueMenu("AngleToAirship", 60.0);
        FrcValueMenu orientedAirshipMoveDistanceMenu = new FrcValueMenu("OrientedAirshipMoveDistance", 5.0);
        FrcValueMenu sidewaysDistanceMenu = new FrcValueMenu("SidewaysDistance", 5.0);
        FrcValueMenu loadingStationTurnAngleMenu = new FrcValueMenu("LoadingStationTurnAngle", 45.0);
        FrcValueMenu loadingStationMoveDistanceMenu = new FrcValueMenu("LoadingStationMoveDistance", 20.0);
        
        //
        // Convert all distances to the unit of inches.
        //
        sideMoveDistance = sideMoveDistanceMenu.getCurrentValue()*12.0;
        angleToAirship = Math.abs(angleToAirshipMenu.getCurrentValue());
        orientedAirshipMoveDistance =  Math.abs(orientedAirshipMoveDistanceMenu.getCurrentValue())*12.0;
        sidewaysDistance = Math.abs(sidewaysDistanceMenu.getCurrentValue())*12.0;
        loadingStationTurnAngle = Math.abs(loadingStationTurnAngleMenu.getCurrentValue());
        loadingStationMoveDistance  = loadingStationMoveDistanceMenu.getCurrentValue()*12.0;
        
        visionDeploy = new CmdVisionGearDeploy(robot);

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
        robot.dashboard.displayPrintf(1, "State: %s", state != null? state.toString(): "Disabled");

        if (sm.isReady())
        {
            boolean printStateInfo = true;
            state = sm.getState();
            double xDistance, yDistance;

            switch (state)
            {
                case DO_DELAY:
                    //
                    // Do delay if any.
                    //
                    if (delay == 0.0)
                    {
                        sm.setState(State.MOVE_FORWARD_ON_SIDE);
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.MOVE_FORWARD_ON_SIDE);
                    }
                    break;

                case MOVE_FORWARD_ON_SIDE:
                    //
                    // Drive the set distance and heading.
                    //
                	xDistance = 0;
                	yDistance = sideMoveDistance;
                	
                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_TOWARDS_AIRSHIP);
                    break;
                    
                case TURN_TOWARDS_AIRSHIP:
                    //
                    // Turn to face airship.
                    //
                	xDistance = 0;
                	yDistance = 0;
                	
                	robot.targetHeading = rightSide ? -angleToAirship : angleToAirship;
                	
                	robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                	sm.waitForSingleEvent(event, State.MOVE_TOWARDS_AIRSHIP);
                	break;
                
                case MOVE_TOWARDS_AIRSHIP:
                    //
                    // Proceed towards gear peg on corresponding side.
                    //
                	xDistance = 0;
                	yDistance = orientedAirshipMoveDistance;

                	robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                	sm.waitForSingleEvent(event, State.VISION_DEPLOY);
                	break;
                
                case VISION_DEPLOY:
                    //
                    // Execute visionDeploy to dispense gear on peg
                    //
                	
                	if (visionDeploy.cmdPeriodic(elapsedTime))
                	{
                		sm.setState(State.BACKUP_FROM_AIRSHIP);
                	}
                	break;
                case BACKUP_FROM_AIRSHIP:
                    //
                    // Backup in addition to backup from visionDeploy
                    //
                	xDistance = 0;
                	yDistance = -orientedAirshipMoveDistance;
                	
                	robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                	sm.waitForSingleEvent(event, State.TURN_TOWARDS_LOADING_STATION);
                	break;
                case TURN_TOWARDS_LOADING_STATION:
                    //
                    // Turn towards loading station after backing up from airship
                    //
                	xDistance = 0;
                	yDistance = 0;
                	
                	robot.targetHeading = rightSide ? loadingStationTurnAngle : -loadingStationTurnAngle;
                	
                	robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                	sm.waitForSingleEvent(event, State.MOVE_TOWARDS_LOADING_STATION);
                	break;
                case MOVE_TOWARDS_LOADING_STATION:
                	//
                	// Move towards (but not in) loading station
                	//
                	xDistance = 0;
                	yDistance = loadingStationMoveDistance;
                	
                	robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
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
            if(printStateInfo)
            {
                robot.traceStateInfo(elapsedTime, state.toString());
            }
        }

        return done;
    }   //cmdPeriodic

}   //class CmdSideGearLift
