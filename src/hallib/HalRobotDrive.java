/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

package hallib;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import trclib.TrcDbgTrace;
import trclib.TrcMotor;
import trclib.TrcUtil;

/**
 * This class implements a robot drive base that supports 2-motor or 4-motor
 * drive trains. It supports tank drive, arcade drive, mecanum drive and swerve
 * drive. This is a port from the WPILib RobotDrive class and extended with
 * addition features.
 */
public class HalRobotDrive extends RobotDrive
{
    private static final String moduleName = "HalRobotDrive";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private static double MOTOR_MAX_VALUE = 1.0;
    private static double MOTOR_MIN_VALUE = -1.0;

    private void commonInit(TrcMotor frontLeftMotor, TrcMotor rearLeftMotor,
                            TrcMotor frontRightMotor, TrcMotor rearRightMotor)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }
    }   //commonInit

    public HalRobotDrive(int leftMotorChannel, int rightMotorChannel)
    {
        super(leftMotorChannel, rightMotorChannel);
        commonInit(null, null, null, null);
    }   //HalRobotDrive

    public HalRobotDrive(int frontLeftMotor, int rearLeftMotor, int frontRightMotor, int rearRightMotor)
    {
        super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
        commonInit(null, null, null, null);
    }   //HalRobotDrive

    public HalRobotDrive(TrcMotor leftMotor, TrcMotor rightMotor)
    {
        super((SpeedController)leftMotor, (SpeedController)rightMotor);
        commonInit(null, leftMotor, null, rightMotor);
    }   //HalRobotDrive

    public HalRobotDrive(TrcMotor frontLeftMotor, TrcMotor rearLeftMotor,
                         TrcMotor frontRightMotor, TrcMotor rearRightMotor)
    {
        super((SpeedController)frontLeftMotor,
              (SpeedController)rearLeftMotor,
              (SpeedController)frontRightMotor,
              (SpeedController)rearRightMotor);
        commonInit(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    }   //HalRobotDrive

    public HalRobotDrive(TrcMotor frontLeftMotor, TrcMotor midLeftMotor, TrcMotor rearLeftMotor,
                         TrcMotor frontRightMotor, TrcMotor midRightMotor, TrcMotor rearRightMotor)
    {
        super((SpeedController)frontLeftMotor, (SpeedController)rearLeftMotor,
              (SpeedController)frontRightMotor, (SpeedController)rearRightMotor);
        throw new UnsupportedOperationException("6-wheel drive is not supported.");
    }   //HalRobotDrive

    /**
     * This method implements tank drive where leftPower controls the left motors
     * and right power controls the right motors.
     *
     * @param leftPower specifies left power value.
     * @param rightPower specifies right power value.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     * @param squaredInput specifies true to square the input values, false otherwise.
     */
    private void tankDrive(double leftPower, double rightPower, boolean inverted, boolean squaredInput)
    {
        if (inverted)
        {
            double swap = leftPower;
            leftPower = -rightPower;
            rightPower = -swap;
        }

        //
        // Somebody at WPI got really confused by motor directions. They decided to negate
        // the right motors. Why??? Any case, we will negate it again here to correct it.
        //
        super.tankDrive(leftPower, -rightPower, squaredInput);
    }   //tankDrive

    /**
     * This method implements tank drive where leftPower controls the left motors
     * and right power controls the right motors.
     *
     * @param leftPower specifies left power value.
     * @param rightPower specifies right power value.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    @Override
    public void tankDrive(double leftPower, double rightPower, boolean inverted)
    {
        tankDrive(leftPower, rightPower, inverted, false);
    }   //tankDrive

    @Override
    public void tankDrive(double leftPower, double rightPower)
    {
        tankDrive(leftPower, rightPower, false, false);
    }   //tankDrive

    public void tankDrive(GenericHID leftStick, int leftAxis, GenericHID rightStick, int rightAxis,
                          boolean inverted, boolean squaredInput)
    {
        tankDrive(leftStick.getRawAxis(leftAxis), rightStick.getRawAxis(rightAxis), inverted, squaredInput);
    }   //tankDrive

    @Override
    public void tankDrive(GenericHID leftStick, int leftAxis, GenericHID rightStick, int rightAxis, boolean inverted)
    {
        tankDrive(leftStick.getRawAxis(leftAxis), rightStick.getRawAxis(rightAxis), inverted, false);
    }   //tankDrive

    @Override
    public void tankDrive(GenericHID leftStick, int leftAxis, GenericHID rightStick, int rightAxis)
    {
        tankDrive(leftStick.getRawAxis(leftAxis), rightStick.getRawAxis(rightAxis), false, false);
    }   //tankDrive

    public void tankDrive(GenericHID leftStick, GenericHID rightStick, boolean inverted, boolean squaredInput)
    {
        tankDrive(leftStick, Joystick.AxisType.kY.value, rightStick, Joystick.AxisType.kY.value,
                  inverted, squaredInput);
    }   //tankDrive

    @Override
    public void tankDrive(GenericHID leftStick, GenericHID rightStick, boolean inverted)
    {
        tankDrive(leftStick, Joystick.AxisType.kY.value, rightStick, Joystick.AxisType.kY.value, inverted, false);
    }   //tankDrive

    @Override
    public void tankDrive(GenericHID leftStick, GenericHID rightStick)
    {
        tankDrive(leftStick, Joystick.AxisType.kY.value, rightStick, Joystick.AxisType.kY.value, false, false);
    }   //tankDrive

    /**
     * This method implements arcade drive where drivePower controls how fast
     * the robot goes in the y-axis and turnPower controls how fast it will
     * turn.
     *
     * @param drivePower specifies the drive power value.
     * @param turnPower specifies the turn power value.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     * @param squaredInput specifies true to square the input values, false otherwise.
     */
    /*
    public void arcadeDrive(
            double drivePower, double turnPower, boolean inverted, boolean squaredInput)
    {
        if (inverted)
        {
            drivePower = -drivePower;
        }
        
        super.arcadeDrive(drivePower, -turnPower, squaredInput);
    }   //arcadeDrive
    */
    public void arcadeDrive(double drivePower, double turnPower, boolean inverted, boolean squaredInput)
    {
        final String funcName = "arcadeDrive";
        double leftPower;
        double rightPower;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                                "drivePower=%f,turnPower=%f,inverted=%s",
                                drivePower, turnPower, Boolean.toString(inverted));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        drivePower = TrcUtil.clipRange(drivePower);
        turnPower = TrcUtil.clipRange(turnPower);

        if (drivePower + turnPower > MOTOR_MAX_VALUE)
        {
            //
            // Forward right:
            //  left = drive + turn - (drive + turn - MOTOR_MAX_VALUE)
            //  right = drive - turn - (drive + turn - MOTOR_MAX_VALUE)
            //
            leftPower = MOTOR_MAX_VALUE;
            rightPower = -2*turnPower + MOTOR_MAX_VALUE;
        }
        else if (drivePower - turnPower > MOTOR_MAX_VALUE)
        {
            //
            // Forward left:
            //  left = drive + turn - (drive - turn - MOTOR_MAX_VALUE)
            //  right = drive - turn - (drive - turn - MOTOR_MAX_VALUE)
            //
            leftPower = 2*turnPower + MOTOR_MAX_VALUE;
            rightPower = MOTOR_MAX_VALUE;
        }
        else if (drivePower + turnPower < MOTOR_MIN_VALUE)
        {
            //
            // Backward left:
            //  left = drive + turn - (drive + turn - MOTOR_MIN_VALUE)
            //  right = drive - turn - (drive + turn - MOTOR_MIN_VALUE)
            //
            leftPower = MOTOR_MIN_VALUE;
            rightPower = -2*turnPower + MOTOR_MIN_VALUE;
        }
        else if (drivePower - turnPower < MOTOR_MIN_VALUE)
        {
            //
            // Backward right:
            //  left = drive + turn - (drive - turn - MOTOR_MIN_VALUE)
            //  right = drive - turn - (drive - turn - MOTOR_MIN_VALUE)
            //
            leftPower = 2*turnPower + MOTOR_MIN_VALUE;
            rightPower = MOTOR_MIN_VALUE;
        }
        else
        {
            leftPower = drivePower + turnPower;
            rightPower = drivePower - turnPower;
        }
        
        tankDrive(leftPower, rightPower, inverted, squaredInput);
    }   //arcadeDrive

    /**
     * This method implements arcade drive where drivePower controls how fast
     * the robot goes in the y-axis and turnPower controls how fast it will
     * turn.
     *
     * @param drivePower specifies the drive power value.
     * @param turnPower specifies the turn power value.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    @Override
    public void arcadeDrive(double drivePower, double turnPower, boolean inverted)
    {
        arcadeDrive(drivePower, turnPower, inverted, false);
    }   //arcadeDrive

    /**
     * This method implements arcade drive where drivePower controls how fast
     * the robot goes in the y-axis and turnPower controls how fast it will
     * turn.
     *
     * @param drivePower specifies the drive power value.
     * @param turnPower specifies the turn power value.
     */
    @Override
    public void arcadeDrive(double drivePower, double turnPower)
    {
        arcadeDrive(drivePower, turnPower, false, false);
    }   //arcadeDrive

    public void arcadeDrive(GenericHID driveStick, int driveAxis, GenericHID turnStick, int turnAxis,
                            boolean inverted, boolean squaredInput)
    {
        arcadeDrive(driveStick.getRawAxis(driveAxis), turnStick.getRawAxis(turnAxis), inverted, squaredInput);
    }   //arcadeDrive
    
    @Override
    public void arcadeDrive(GenericHID driveStick, int driveAxis, GenericHID turnStick, int turnAxis, boolean inverted)
    {
        arcadeDrive(driveStick.getRawAxis(driveAxis), turnStick.getRawAxis(turnAxis), inverted, false);
    }   //arcadeDrive
    
    @Override
    public void arcadeDrive(GenericHID driveStick, int driveAxis, GenericHID turnStick, int turnAxis)
    {
        arcadeDrive(driveStick.getRawAxis(driveAxis), turnStick.getRawAxis(turnAxis), false, false);
    }   //arcadeDrive
    
    public void arcadeDrive(GenericHID stick, boolean inverted, boolean squaredInput)
    {
        arcadeDrive(stick.getRawAxis(Joystick.AxisType.kY.value), stick.getRawAxis(Joystick.AxisType.kX.value),
                    inverted, squaredInput);
    }   //arcadeDrive

    @Override
    public void arcadeDrive(GenericHID stick, boolean inverted)
    {
        arcadeDrive(stick.getRawAxis(Joystick.AxisType.kY.value), stick.getRawAxis(Joystick.AxisType.kX.value),
                    inverted, false);
    }   //arcadeDrive
    
    @Override
    public void arcadeDrive(GenericHID stick)
    {
        arcadeDrive(stick.getRawAxis(Joystick.AxisType.kY.value), stick.getRawAxis(Joystick.AxisType.kX.value),
                    false, false);
    }   //arcadeDrive
    
    /**
     * This method implements mecanum drive where x controls how fast the robot will
     * go in the x direction, and y controls how fast the robot will go in the y direction.
     * Rotation controls how fast the robot rotates and gyroAngle specifies the heading
     * the robot should maintain.
     * @param x specifies the x power.
     * @param y specifies the y power.
     * @param rotation specifies the rotating power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     * @param gyroAngle specifies the gyro angle to maintain.
     */
    public void mecanumDrive_Cartesian(double x, double y, double rotation, boolean inverted, double gyroAngle)
    {
        if (inverted)
        {
            x = -x;
            y = -y;
        }
        
        super.mecanumDrive_Cartesian(x, -y, rotation, gyroAngle);
    }   //mecanumDrive_Cartesian

    /**
     * This method implements mecanum drive where x controls how fast the robot will
     * go in the x direction, and y controls how fast the robot will go in the y direction.
     * Rotation controls how fast the robot rotates and gyroAngle specifies the heading
     * the robot should maintain.
     * @param x specifies the x power.
     * @param y specifies the y power.
     * @param rotation specifies the rotating power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void mecanumDrive_Cartesian(double x, double y, double rotation, boolean inverted)
    {
        mecanumDrive_Cartesian(x, y, rotation, inverted, 0.0);
    }   //mecanumDrive_Cartesian

    /**
     * This method implements mecanum drive where x controls how fast the robot will
     * go in the x direction, and y controls how fast the robot will go in the y direction.
     * Rotation controls how fast the robot rotates and gyroAngle specifies the heading
     * the robot should maintain.
     * @param x specifies the x power.
     * @param y specifies the y power.
     * @param rotation specifies the rotating power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void mecanumDrive_Cartesian(double x, double y, double rotation)
    {
        mecanumDrive_Cartesian(x, y, rotation, false, 0.0);
    }   //mecanumDrive_Cartesian

    /**
     * This method implements mecanum drive where x controls how fast the robot will
     * go in the x direction, and y controls how fast the robot will go in the y direction.
     * Rotation controls how fast the robot rotates and gyroAngle specifies the heading
     * the robot should maintain.
     * @param x specifies the x power.
     * @param y specifies the y power.
     * @param rotation specifies the rotating power.
     * @param gyroAngle specifies the gyro angle to maintain.
     */
    @Override
    public void mecanumDrive_Cartesian(double x, double y, double rotation, double gyroAngle)
    {
        mecanumDrive_Cartesian(x, y, rotation, false, gyroAngle);
    }   //mecanumDrive_Cartesian

    /**
     * This method implements mecanum drive where magnitude controls how fast the robot
     * will go in the given direction and how fast it will robote.
     *
     * @param magnitude specifies the magnitude combining x and y axes.
     * @param direction specifies the direction in degrees.
     * @param rotation specifies the rotation power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void mecanumDrive_Polar(double magnitude, double direction, double rotation, boolean inverted)
    {
        if (inverted)
        {
            direction += 180.0;
            direction %= 360.0;
        }

        super.mecanumDrive_Polar(magnitude, direction, rotation);
    }   //mecanumDrive_Polar

}   //HalRobotDrive
