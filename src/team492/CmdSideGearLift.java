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
import team492.Robot.Alliance;
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
        INITIAL_TURN_TOWARDS_AIRSHIP,
        SLOW_TURN_TOWARDS_AIRSHIP,
        VISION_DEPLOY,
        BACKUP_FROM_AIRSHIP,
        TURN_TOWARDS_LOADING_STATION,
        MOVE_TOWARDS_LOADING_STATION,
        MOVE_PAST_AIRSHIP,
        TURN_ALONG_DIAGONAL,
        MOVE_ALONG_DIAGONAL,
        DONE
    }   //enum State

    private static final String moduleName = "CmdSideGearLift";

    private Robot robot;
    private double delay;
    private boolean rightSide;
    private boolean isRed;
    private double sideAirshipDistance;
    private double startSideLiftAngle;
    private double sideLiftAngleIncrement;
    private double maxSideLiftAngle;
    private double orientedAirshipMoveDistance;
    private double loadingStationTurnAngle;
    private double loadingStationMoveDistance;
    private double baseLineToLaunchPadLine;
    private double diagonalTurnAngle;
    private double diagonalDistance;

    private CmdVisionGearDeploy visionDeploy;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;

    CmdSideGearLift(Robot robot, double delay, boolean rightSide)
    {
        this.robot = robot;
        this.delay = delay;
        this.rightSide = rightSide;
        this.isRed = robot.alliance == Alliance.RED_ALLIANCE;

        //
        // Convert all distances to the unit of inches.
        //
        sideAirshipDistance = HalDashboard.getNumber("SideAirshipDistance", 48.0);
        startSideLiftAngle = Math.abs(HalDashboard.getNumber("SideLiftAngle", 45.0));
        sideLiftAngleIncrement = Math.abs(HalDashboard.getNumber("SideLiftAngleIncrement", 5.0));
        maxSideLiftAngle = Math.abs(HalDashboard.getNumber("SideLiftMaxAngle", 60.0));
        orientedAirshipMoveDistance = HalDashboard.getNumber("SideOrientedAirshipDistance", 30.0);
        loadingStationTurnAngle = Math.abs(HalDashboard.getNumber("SideLoadingStationAngle", 0.0));
        loadingStationMoveDistance = HalDashboard.getNumber("SideLoadingStationDistance", 40.0*12.0);
        baseLineToLaunchPadLine = HalDashboard.getNumber("SideBaselineToLaunchpadLine",92); //from schematics
        diagonalTurnAngle = HalDashboard.getNumber("SideDiagonalAngle", 50); // 50 from calculations with schematics
        diagonalDistance = HalDashboard.getNumber("SideDiagonalDistance", 350); // 350 from calculations with schematics

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
                    yDistance = sideAirshipDistance;

                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.INITIAL_TURN_TOWARDS_AIRSHIP);
                    break;

                case INITIAL_TURN_TOWARDS_AIRSHIP:
                    //
                    // Turn to face airship.
                    //
                    xDistance = 0;
                    yDistance = 0;
                    robot.targetHeading = rightSide ? -startSideLiftAngle : startSideLiftAngle;

                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.SLOW_TURN_TOWARDS_AIRSHIP);
                    break;

                case SLOW_TURN_TOWARDS_AIRSHIP:
                    //
                    // Turn until vision target visible
                    //
                    xDistance = 0;
                    yDistance = 0;
                    // If vision target detected, or if we hit the max turn angle, set the next state
                    if (robot.frontPixy.getTargetInfo() != null || Math.abs(robot.targetHeading) >= maxSideLiftAngle)
                    {
                        sm.setState(State.VISION_DEPLOY);
                    }
                    else
                    {
                        // The vision target isn't visible, increment the targetHeading.
                        robot.targetHeading += rightSide?-sideLiftAngleIncrement:sideLiftAngleIncrement;
                        // Turn to the new target heading
                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                        sm.waitForSingleEvent(event, State.SLOW_TURN_TOWARDS_AIRSHIP);
                    }
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
                    // MTS: loadingStationTurnAnlge might just be ZERO!!!
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
                    if((isRed && rightSide) || (!isRed && !rightSide))
                    {
                        sm.setState(State.MOVE_PAST_AIRSHIP);
                    }
                    else
                    {
                        xDistance = 0;
                        yDistance = loadingStationMoveDistance;

                        robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                        sm.waitForSingleEvent(event, State.DONE);
                    }
                    break;

                case MOVE_PAST_AIRSHIP:
                    //
                    // Move from baseline to launchpad line
                    //
                    xDistance = 0;
                    yDistance = baseLineToLaunchPadLine;

                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_ALONG_DIAGONAL);
                    break;

                case TURN_ALONG_DIAGONAL:
                    //
                    // Turn to the opposite corner of neutral zone
                    //
                    xDistance = 0;
                    yDistance = 0;
                    robot.targetHeading = diagonalTurnAngle * (isRed?-1:1);

                    robot.pidDrive.setTarget(xDistance, yDistance, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.MOVE_ALONG_DIAGONAL);
                    break;

                case MOVE_ALONG_DIAGONAL:
                    //
                    // Move to the opposite corner of neutral zone
                    //
                    xDistance = 0;
                    yDistance = diagonalDistance;

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
