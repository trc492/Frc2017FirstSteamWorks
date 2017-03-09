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

import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdMidGearLift implements TrcRobot.RobotCommand
{
    private static enum State
    {
        DO_DELAY,
        GO_TO_MID_LIFT,
        VISION_GEAR_DEPLOY,
        TURN_TO_SIDEWALL,
        GO_TO_SIDEWALL,
        TURN_TO_OPPOSITE_END,
        GO_TO_OPPOSITE_END,
        TURN_TO_LOADING_STATION,
        DONE
    }   //enum State

    private static final String moduleName = "CmdMidGearLift";

    private Robot robot;
    private double delay;
    private double midLiftDistance;
    private double midSidewallAngle;
    private double midSidewallDistance;
    private double midOppositeEndDistance;
    private double midLoadingStationAngle;
    private CmdVisionGearDeploy cmdVisionDeploy;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;

    CmdMidGearLift(Robot robot, double delay)
    {
        this.robot = robot;
        this.delay = delay;

        //
        // Get parameters from the SmartDashboard. 
        //

        // approx. 33.3 inches.
        midLiftDistance = HalDashboard.getNumber("MidLiftDistance", 36.0);
        midSidewallAngle = Math.abs(HalDashboard.getNumber("MidSidewallAngle", 90));
        // approx. 80 inches.
        midSidewallDistance = HalDashboard.getNumber("MidSidewallDistance", 72.0);//RobotInfo.FIELD_AIRSHIP_PANEL_WIDTH*2.0);
        midOppositeEndDistance = HalDashboard.getNumber("MidOppositeEndDistance", 33.0*12.0);
        midLoadingStationAngle = Math.abs(HalDashboard.getNumber("MidLoadingStationAngle", 0.0));

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
                        sm.setState(State.GO_TO_MID_LIFT);
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.GO_TO_MID_LIFT);
                    }
                    break;

                case GO_TO_MID_LIFT:
                    //
                    // Go towards the mid lift.
                    //
                    xDistance = 0.0;
                    yDistance = midLiftDistance;

                    robot.setPidDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.VISION_GEAR_DEPLOY);
                    break;

                case VISION_GEAR_DEPLOY:
                    //
                    // Have VisionGearDeploy aligning to the peg, deploy the gear and back up.
                    //
                    printStateInfo = false;
                    if (cmdVisionDeploy.cmdPeriodic(elapsedTime))
                    {
                        sm.setState(State.TURN_TO_SIDEWALL);
                    }
                    break;

                case TURN_TO_SIDEWALL:
                    //
                    // Turn towards the loading station side wall.
                    //
                    xDistance = yDistance = 0.0;
                    robot.targetHeading =
                        robot.alliance == Robot.Alliance.RED_ALLIANCE? -midSidewallAngle: midSidewallAngle; 

                    robot.setPidDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.GO_TO_SIDEWALL);
                    break;

                case GO_TO_SIDEWALL:
                    //
                    // Move towards the loading station side wall.
                    //
                    xDistance = 0.0;
                    yDistance = midSidewallDistance;

//                    robot.encoderYPidCtrl.setOutputRange(-1.0, 1.0);
                    robot.setPidDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_TO_OPPOSITE_END);
                    break;

                case TURN_TO_OPPOSITE_END:
                    //
                    // Turn heading the opposite end.
                    //
                    xDistance = yDistance = 0.0;
                    robot.targetHeading = 0.0;

                    robot.setPidDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.GO_TO_OPPOSITE_END);
                    break;

                case GO_TO_OPPOSITE_END:
                    //
                    // Move towards the opposite end.
                    //
                    xDistance = 0.0;
                    yDistance = midOppositeEndDistance;

                    robot.setPidDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_TO_LOADING_STATION);
                    break;

                case TURN_TO_LOADING_STATION:
                    //
                    // Turn towards the loading station.
                    //
                    xDistance = yDistance = 0.0;
                    robot.targetHeading = midLoadingStationAngle;

                    robot.setPidDriveTarget(xDistance, yDistance, robot.targetHeading, false, event);
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
