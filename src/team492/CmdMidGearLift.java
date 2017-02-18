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

class CmdMidGearLift implements TrcRobot.RobotCommand
{
    private static enum State
    {
        DO_DELAY,
        GOTO_MID_LIFT,
        VISION_GEAR_DEPLOY,
        MOVE_SIDEWAYS,
        GOTO_RETRIEVAL_ZONE,
        TURNTO_LOADING_STATION,
        DONE
    }   //enum State

    private static final String moduleName = "CmdMidGearLift";

    private Robot robot;
    private double delay;
    private double midLiftDistance;
    private double sidewaysDistance;
    private double retrievalZoneDistance;
    private double loadingStationAngle;
    private CmdVisionGearDeploy cmdVisionDeploy;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;

    CmdMidGearLift(Robot robot, double delay)
    {
        this.robot = robot;
        this.delay = delay;

        //
        // All distances from the menus are in the unit of feet.
        //
        FrcValueMenu midLiftDistanceMenu = new FrcValueMenu("MidLiftDistance", 7.0 - RobotInfo.ROBOT_LENGTH/12.0);
        FrcValueMenu sidewaysDistanceMenu = new FrcValueMenu("SidewaysDistance", 5.0);
        FrcValueMenu retrievalZoneDistanceMenu = new FrcValueMenu("RetrievalZoneDistance", 40.0);
        FrcValueMenu loadingStationAngleMenu = new FrcValueMenu("LoadingStationAngle", 45.0);

        //
        // Convert all distances to the unit of inches.
        //
        midLiftDistance = midLiftDistanceMenu.getCurrentValue()*12.0;
        sidewaysDistance = Math.abs(sidewaysDistanceMenu.getCurrentValue()*12.0);
        retrievalZoneDistance = retrievalZoneDistanceMenu.getCurrentValue()*12.0;
        loadingStationAngle = loadingStationAngleMenu.getCurrentValue()*12.0;

        cmdVisionDeploy = new CmdVisionGearDeploy(robot);

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
                        sm.setState(State.GOTO_MID_LIFT);
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.GOTO_MID_LIFT);
                    }
                    break;

                case GOTO_MID_LIFT:
                    //
                    // Go towards the mid lift.
                    //
                    xDistance = 0.0;
                    yDistance = midLiftDistance;

                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.VISION_GEAR_DEPLOY);
                    break;

                case VISION_GEAR_DEPLOY:
                    //
                    // Have VisionGearDeploy aligning to the peg, deploy the gear and back up.
                    //
                    printStateInfo = false;
                    if (cmdVisionDeploy.cmdPeriodic(elapsedTime))
                    {
                        sm.setState(State.MOVE_SIDEWAYS);
                    }
                    break;

                case MOVE_SIDEWAYS:
                    //
                    // Move sideways towards the loading station side and clear of the airship.
                    //
                    xDistance = robot.alliance == Robot.Alliance.RED_ALLIANCE? -sidewaysDistance: sidewaysDistance;
                    yDistance = 0.0;

                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.GOTO_RETRIEVAL_ZONE);
                    break;

                case GOTO_RETRIEVAL_ZONE:
                    //
                    // Move towards the retrieval zone.
                    //
                    xDistance = 0.0;
                    yDistance = retrievalZoneDistance;

                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.TURNTO_LOADING_STATION);
                    break;

                case TURNTO_LOADING_STATION:
                    //
                    // Turn towards the loading station.
                    //
                    xDistance = yDistance = 0.0;
                    robot.targetHeading = robot.alliance == Robot.Alliance.RED_ALLIANCE?
                        -loadingStationAngle: loadingStationAngle;

                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);

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
