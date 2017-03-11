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

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frclib.FrcChoiceMenu;
import trclib.TrcEvent;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

public class FrcTest extends FrcTeleOp
{
    private static final String moduleName = "FrcTest";

    public enum Test
    {
        SENSORS_TEST,
        DRIVE_MOTORS_TEST,
        X_TIMED_DRIVE,
        Y_TIMED_DRIVE,
        X_DISTANCE_DRIVE,
        Y_DISTANCE_DRIVE,
        TURN_DEGREES,
        LIVE_WINDOW
    }   //enum Test

    private enum State
    {
        START,
        DONE
    }   //State

    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;

    //
    // Test choice menu.
    //
    private FrcChoiceMenu<Test> testMenu;
    private Test test;

    private CmdTimedDrive timedDriveCommand = null;
    private CmdPidDrive pidDriveCommand = null;

    private int motorIndex = 0;

    public FrcTest(Robot robot)
    {
        //
        // Call TeleOp constructor.
        //
        super(robot);

        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        //
        // Create and populate Test Mode specific menus.
        //
        testMenu = new FrcChoiceMenu<>("Tests");
        testMenu.addChoice("Sensors Test", FrcTest.Test.SENSORS_TEST, true);
        testMenu.addChoice("Drive Motors Test", FrcTest.Test.DRIVE_MOTORS_TEST, false);
        testMenu.addChoice("X Timed Drive", FrcTest.Test.X_TIMED_DRIVE, false);
        testMenu.addChoice("Y Timed Drive", FrcTest.Test.Y_TIMED_DRIVE, false);
        testMenu.addChoice("X Distance Drive", FrcTest.Test.X_DISTANCE_DRIVE, false);
        testMenu.addChoice("Y Distance Drive", FrcTest.Test.Y_DISTANCE_DRIVE, false);
        testMenu.addChoice("Turn Degrees", FrcTest.Test.TURN_DEGREES, false);
        testMenu.addChoice("Live Window", FrcTest.Test.LIVE_WINDOW, false);
     }   //FrcTest

    //
    // Overriding TrcRobot.RobotMode.
    //

    @Override
    public void startMode()
    {
        if (Robot.USE_TRACELOG) robot.startTraceLog();
        //
        // Call TeleOp startMode.
        //
        super.startMode();

        //
        // Retrieve menu choice values.
        //
        test = testMenu.getCurrentChoiceObject();

        boolean liveWindowEnabled = false;
        switch (test)
        {
            case DRIVE_MOTORS_TEST:
                motorIndex = 0;
                break;

            case X_TIMED_DRIVE:
                timedDriveCommand = new CmdTimedDrive(robot, 0.0, robot.driveTime, robot.drivePower, 0.0, 0.0);
                break;

            case Y_TIMED_DRIVE:
                timedDriveCommand = new CmdTimedDrive(robot, 0.0, robot.driveTime, 0.0, robot.drivePower, 0.0);
                break;

            case X_DISTANCE_DRIVE:
                pidDriveCommand = new CmdPidDrive(robot, 0.0, robot.driveDistance, 0.0, 0.0, robot.drivePowerLimit);
                break;

            case Y_DISTANCE_DRIVE:
                pidDriveCommand = new CmdPidDrive(robot, 0.0, 0.0, robot.driveDistance, 0.0, robot.drivePowerLimit);
                break;

            case TURN_DEGREES:
                pidDriveCommand = new CmdPidDrive(robot, 0.0, 0.0, 0.0, robot.turnDegrees, robot.drivePowerLimit);
                break;

            case LIVE_WINDOW:
                liveWindowEnabled = true;
                break;

            default:
                break;
        }

        LiveWindow.setEnabled(liveWindowEnabled);
        sm.start(State.START);
    }   //startMode

    @Override
    public void stopMode()
    {
        //
        // Call TeleOp stopMode.
        //
        super.stopMode();
        if (Robot.USE_TRACELOG) robot.stopTraceLog();
    }   //stopMode

    //
    // Must override TeleOp so it doesn't fight with us.
    //
    @Override
    public void runPeriodic(double elapsedTime)
    {
        switch (test)
        {
            case SENSORS_TEST:
                //
                // Allow TeleOp to run so we can control the robot in sensors test mode.
                //
                super.runPeriodic(elapsedTime);
                doSensorsTest();
                break;

            case DRIVE_MOTORS_TEST:
                doDriveMotorsTest();
                break;

            case LIVE_WINDOW:
                LiveWindow.run();
                break;

            default:
                break;
        }
    }   //runPeriodic

    @Override
    public void runContinuous(double elapsedTime)
    {
        State state = sm.getState();
        robot.dashboard.displayPrintf(1, "%s: %s", test.toString(), state != null? state.toString(): "STOPPED!");

        switch (test)
        {
            case X_TIMED_DRIVE:
            case Y_TIMED_DRIVE:
                double lfEnc = robot.leftFrontWheel.getPosition();
                double rfEnc = robot.rightFrontWheel.getPosition();
                double lrEnc = robot.leftRearWheel.getPosition();
                double rrEnc = robot.rightRearWheel.getPosition();
                robot.dashboard.displayPrintf(2, "Enc:lf=%.0f,rf=%.0f", lfEnc, rfEnc);
                robot.dashboard.displayPrintf(3, "Enc:lr=%.0f,rr=%.0f", lrEnc, rrEnc);
                robot.dashboard.displayPrintf(4, "average=%f", (lfEnc + rfEnc + lrEnc + rrEnc)/4.0);
                robot.dashboard.displayPrintf(5, "xPos=%.1f,yPos=%.1f,heading=%.1f",
                    robot.driveBase.getXPosition(), robot.driveBase.getYPosition(), robot.driveBase.getHeading());
                timedDriveCommand.cmdPeriodic(elapsedTime);
                break;

            case X_DISTANCE_DRIVE:
            case Y_DISTANCE_DRIVE:
            case TURN_DEGREES:
                robot.dashboard.displayPrintf(2, "xPos=%.1f,yPos=%.1f,heading=%.1f",
                    robot.getInput(robot.encoderXPidCtrl), robot.getInput(robot.encoderYPidCtrl),
                    robot.getInput(robot.gyroTurnPidCtrl));
                robot.encoderXPidCtrl.displayPidInfo(3);
                robot.encoderYPidCtrl.displayPidInfo(5);
                robot.gyroTurnPidCtrl.displayPidInfo(7);

                if (!pidDriveCommand.cmdPeriodic(elapsedTime))
                {
                    if (test == Test.X_DISTANCE_DRIVE)
                    {
                        robot.encoderXPidCtrl.printPidInfo(robot.tracer);
                    }
                    else if (test == Test.Y_DISTANCE_DRIVE)
                    {
                        robot.encoderYPidCtrl.printPidInfo(robot.tracer);
                    }
                    else if (test == Test.TURN_DEGREES)
                    {
                        robot.gyroTurnPidCtrl.printPidInfo(robot.tracer);
                    }
                }
                break;

            default:
                break;
        }
    }   //runContinuous

    /**
     * This method reads all sensors and prints out their values. This is a very useful diagnostic tool to check
     * if all sensors are working properly. For encoders, since test sensor mode is also teleop mode, you can
     * operate the gamepads to turn the motors and check the corresponding encoder counts.
     */
    private void doSensorsTest()
    {
        robot.dashboard.displayPrintf(2, "DriveBase: lf=%.0f, rf=%.0f, lr=%.0f, rr=%.0f",
            robot.leftFrontWheel.getPosition(), robot.rightFrontWheel.getPosition(),
            robot.leftRearWheel.getPosition(), robot.rightRearWheel.getPosition());
        robot.dashboard.displayPrintf(3, "DriveBase: X=%.1f, Y=%.1f, Heading=%.1f",
            robot.driveBase.getXPosition(), robot.driveBase.getYPosition(), robot.driveBase.getHeading());
        robot.dashboard.displayPrintf(4, "Gyro: Rate=%.3f, Heading=%.1f",
            robot.gyro.getZRotationRate().value, robot.gyro.getZHeading().value);
        robot.dashboard.displayPrintf(5, "Winch: position=%.1f, touch=%s/%s",
            robot.winch.getPosition(), Boolean.toString(robot.winch.isFwdLimitSwitchActive()),
            Boolean.toString(robot.winch.isRevLimitSwitchActive()));
        robot.dashboard.displayPrintf(6, "GearPickup: gear=%s", robot.gearPickup.gearDetected());
        robot.dashboard.displayPrintf(7, "PressureSensor: pressure=%.1f", robot.getPressure());
        robot.dashboard.displayPrintf(8, "Ultrasonic=%.1f inches", robot.getUltrasonicDistance());
    }   //doSensorsTest

    /**
     * This method runs each of the four wheels in sequence for a fixed number of seconds. It is for diagnosing
     * problems with the drive train. At the end of the run, you should check the amount of encoder counts each
     * wheel has accumulated. They should be about the same. If not, you need to check the problem wheel for
     * friction or chain tension etc. You can also use this test to check if a motor needs to be "inverted"
     * (i.e. turning in the wrong direction).
     */
    private void doDriveMotorsTest()
    {
        robot.dashboard.displayPrintf(1, "Motors Test: index=%d", motorIndex);
        robot.dashboard.displayPrintf(2, "Enc: lf=%.0f, rf=%.0f, lr=%.0f, rr=%.0f",
            robot.leftFrontWheel.getPosition(), robot.rightFrontWheel.getPosition(),
            robot.leftRearWheel.getPosition(), robot.rightRearWheel.getPosition());

        if (sm.isReady())
        {
            State state = sm.getState();
            switch (state)
            {
                case START:
                    //
                    // Spin a wheel for 5 seconds.
                    //
                    switch (motorIndex)
                    {
                        case 0:
                            //
                            // Run the left front wheel.
                            //
                            robot.leftFrontWheel.setPower(0.5);
                            robot.rightFrontWheel.setPower(0.0);
                            robot.leftRearWheel.setPower(0.0);
                            robot.rightRearWheel.setPower(0.0);
                            break;

                        case 1:
                            //
                            // Run the right front wheel.
                            //
                            robot.leftFrontWheel.setPower(0.0);
                            robot.rightFrontWheel.setPower(0.5);
                            robot.leftRearWheel.setPower(0.0);
                            robot.rightRearWheel.setPower(0.0);
                            break;

                        case 2:
                            //
                            // Run the left rear wheel.
                            //
                            robot.leftFrontWheel.setPower(0.0);
                            robot.rightFrontWheel.setPower(0.0);
                            robot.leftRearWheel.setPower(0.5);
                            robot.rightRearWheel.setPower(0.0);
                            break;

                        case 3:
                            //
                            // Run the right rear wheel.
                            //
                            robot.leftFrontWheel.setPower(0.0);
                            robot.rightFrontWheel.setPower(0.0);
                            robot.leftRearWheel.setPower(0.0);
                            robot.rightRearWheel.setPower(0.5);
                            break;
                    }
                    motorIndex = motorIndex + 1;
                    timer.set(5.0, event);
                    sm.waitForSingleEvent(event, motorIndex < 4? State.START: State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done, stop all wheels.
                    //
                    robot.driveBase.stop();
                    sm.stop();
                    break;
            }
        }
    }   //doMotorsTest

}   //class FrcTest
