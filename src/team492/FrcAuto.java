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

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import hallib.HalDashboard;
import trclib.TrcRobot;

public class FrcAuto implements TrcRobot.RobotMode
{
    public enum AutoMode
    {
        AUTOMODE_TIMED_DRIVE,
        AUTOMODE_PID_DRIVE,
        AUTOMODE_NONE
    }

    private Robot robot;
    private SendableChooser<AutoMode> autoChooser;
    private TrcRobot.RobotCommand autoCommand;

    public FrcAuto(Robot robot)
    {
        this.robot = robot;

        autoChooser = new SendableChooser<>();
        autoChooser.addDefault("No autonomous", AutoMode.AUTOMODE_NONE);
        autoChooser.addDefault("Timed drive", AutoMode.AUTOMODE_TIMED_DRIVE);
        autoChooser.addDefault("PID drive", AutoMode.AUTOMODE_PID_DRIVE);
        HalDashboard.putData("Autonomous Strategies", autoChooser);
     }   //FtcAuto

    //
    // Implements TrcRobot.RunMode.
    //

    @Override
    public void startMode()
    {
        robot.driveBase.resetPosition();

        double delay = 0.0;
        double driveTime = 0.0;
        double xDrivePower = 0.0;
        double yDrivePower = 0.0;
        double turnPower = 0.0;
        double xDistance = 0.0;
        double yDistance = 0.0;
        double heading = 0.0;

        AutoMode selectedAutoMode = autoChooser.getSelected();
        switch (selectedAutoMode)
        {
            case AUTOMODE_TIMED_DRIVE:
                autoCommand = new CmdTimedDrive(robot, delay, driveTime, xDrivePower, yDrivePower, turnPower);
                break;

            case AUTOMODE_PID_DRIVE:
                autoCommand = new CmdPidDrive(robot, delay, xDistance, yDistance, heading);
                break;

            default:
            case AUTOMODE_NONE:
                autoCommand = null;
                break;
        }
    }   //startMode

    @Override
    public void stopMode()
    {
        robot.driveBase.stop();
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
    }   //runPeriodic

    @Override
    public void runContinuous(double elapsedTime)
    {
        if (autoCommand != null)
        {
            autoCommand.cmdPeriodic(elapsedTime);
        }
    }   //runContinuous

}   //class FtcAuto
