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

import frclib.FrcJoystick;
import trclib.TrcRobot;

public class FrcTeleOp implements TrcRobot.RobotMode, FrcJoystick.ButtonHandler
{
    private enum DriveMode
    {
        MECANUM_MODE,
        ARCADE_MODE,
        TANK_MODE
    }   //enum DriveMode

    protected Robot robot;

    //
    // Input subsystem.
    //
    private FrcJoystick leftDriveStick;
    private FrcJoystick rightDriveStick;
    private FrcJoystick operatorStick;

    private boolean slowDriveOverride = false;
    private DriveMode driveMode = DriveMode.MECANUM_MODE;

    public FrcTeleOp(Robot robot)
    {
        this.robot = robot;
        //
        // Input subsystem.
        //
        leftDriveStick = new FrcJoystick("leftDriveStick", RobotInfo.JSPORT_LEFT_DRIVESTICK, this);
        leftDriveStick.setYInverted(true);

        rightDriveStick = new FrcJoystick("rightDriveStick", RobotInfo.JSPORT_RIGHT_DRIVESTICK, this);
        rightDriveStick.setYInverted(true);

        operatorStick = new FrcJoystick("operatorStick", RobotInfo.JSPORT_OPERATORSTICK, this);
        operatorStick.setYInverted(true);
    }   //FrcTeleOp

    //
    // Implements TrcRobot.RunMode interface.
    //

    @Override
    public void startMode()
    {
    }   //startMode

    @Override
    public void stopMode()
    {
        robot.driveBase.stop();
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        //
        // DriveBase operation.
        //
        switch (driveMode)
        {
            case TANK_MODE:
                double leftPower = leftDriveStick.getYWithDeadband(true);
                double rightPower = rightDriveStick.getYWithDeadband(true);
                if (slowDriveOverride)
                {
                    leftPower /= RobotInfo.DRIVE_SLOW_YSCALE;
                    rightPower /= RobotInfo.DRIVE_SLOW_YSCALE;
                }
                robot.driveBase.tankDrive(leftPower, rightPower);
                break;

            case ARCADE_MODE:
                double drivePower = rightDriveStick.getYWithDeadband(true);
                double turnPower = rightDriveStick.getTwistWithDeadband(true);
                if (slowDriveOverride)
                {
                    drivePower /= RobotInfo.DRIVE_SLOW_YSCALE;
                    turnPower /= RobotInfo.DRIVE_SLOW_TURNSCALE;
                }
                robot.driveBase.arcadeDrive(drivePower, turnPower);
                break;

            default:
            case MECANUM_MODE:
                double x = leftDriveStick.getXWithDeadband(true);
                double y = rightDriveStick.getYWithDeadband(true);
                double rot = rightDriveStick.getTwistWithDeadband(true);
                if (slowDriveOverride)
                {
                    x /= RobotInfo.DRIVE_SLOW_XSCALE;
                    y /= RobotInfo.DRIVE_SLOW_YSCALE;
                    rot /= RobotInfo.DRIVE_SLOW_TURNSCALE;
                }
                robot.driveBase.mecanumDrive_Cartesian(x, y, rot);
                break;
        }

        robot.updateDashboard();
    }   //runPeriodic

    @Override
    public void runContinuous(double elapsedTime)
    {
    }   //runContinuous

    //
    // Implements TrcJoystick.ButtonHandler.
    //

    @Override
    public void joystickButtonEvent(FrcJoystick joystick, int button, boolean pressed)
    {
        if (joystick == leftDriveStick)
        {
            switch (button)
            {
                case FrcJoystick.LOGITECH_TRIGGER:
                    break;
    
                case FrcJoystick.LOGITECH_BUTTON2:
                    break;

                case FrcJoystick.LOGITECH_BUTTON3:
                    break;

                case FrcJoystick.LOGITECH_BUTTON4:
                    break;

                case FrcJoystick.LOGITECH_BUTTON5:
                    break;

                case FrcJoystick.LOGITECH_BUTTON6:
                    break;

                case FrcJoystick.LOGITECH_BUTTON7:
                    break;

                case FrcJoystick.LOGITECH_BUTTON8:
                    break;

                case FrcJoystick.LOGITECH_BUTTON9:
                    break;

                case FrcJoystick.LOGITECH_BUTTON10:
                    break;

                case FrcJoystick.LOGITECH_BUTTON11:
                    break;

                case FrcJoystick.LOGITECH_BUTTON12:
                    break;
            }
        }
        else if (joystick == rightDriveStick)
        {
            switch (button)
            {
                case FrcJoystick.SIDEWINDER_TRIGGER:
                    slowDriveOverride = pressed;
                    break;

                case FrcJoystick.SIDEWINDER_BUTTON2:
                    break;

                case FrcJoystick.SIDEWINDER_BUTTON3:
                    break;

                case FrcJoystick.SIDEWINDER_BUTTON4:
                    break;

                case FrcJoystick.SIDEWINDER_BUTTON5:
                    break;

                case FrcJoystick.SIDEWINDER_BUTTON6:
                    break;

                case FrcJoystick.SIDEWINDER_BUTTON7:
                    break;

                case FrcJoystick.SIDEWINDER_BUTTON8:
                    break;

                case FrcJoystick.SIDEWINDER_BUTTON9:
                    break;
            }
        }
        else if (joystick == operatorStick)
        {
            switch (button)
            {
                case FrcJoystick.LOGITECH_TRIGGER:
                    break;

                case FrcJoystick.LOGITECH_BUTTON2:
                    break;

                case FrcJoystick.LOGITECH_BUTTON3:
                    break;

                case FrcJoystick.LOGITECH_BUTTON4:
                    break;

                case FrcJoystick.LOGITECH_BUTTON5:
                    break;

                case FrcJoystick.LOGITECH_BUTTON6:
                    break;

                case FrcJoystick.LOGITECH_BUTTON7:
                    break;

                case FrcJoystick.LOGITECH_BUTTON8:
                    break;

                case FrcJoystick.LOGITECH_BUTTON9:
                    break;

                case FrcJoystick.LOGITECH_BUTTON10:
                    break;

                case FrcJoystick.LOGITECH_BUTTON11:
                    break;

                case FrcJoystick.LOGITECH_BUTTON12:
                    break;
            }
        }
    }   //joystickButtonEvent

}   //class FrcTeleOp
