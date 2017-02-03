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

public class RobotInfo
{
    //
    // Compiler switches
    //
    
    //
    // Joystick ports.
    //
    public static final int JSPORT_LEFT_DRIVESTICK          = 0;
    public static final int JSPORT_RIGHT_DRIVESTICK         = 1;
    public static final int JSPORT_OPERATORSTICK            = 2;

    //
    // Digital Output ports.
    //

    //
    // CAN IDs.
    //
    public static final int CANID_LEFTFRONTWHEEL            = 3;    //40A: Orange
    public static final int CANID_RIGHTFRONTWHEEL           = 4;    //40A: Yellow
    public static final int CANID_LEFTREARWHEEL             = 5;    //40A: Green
    public static final int CANID_RIGHTREARWHEEL            = 6;    //40A: Blue
    
    public static final int CANID_PDP                       = 16;
    public static final int CANID_PCM1                      = 17;
    public static final int CANID_PCM2                      = 18;
    public static final int MAILBOX_EXTEND					= 0;
    public static final int MAILBOX_RETRACT					= 1;
    

    //
    // Relay channels.
    //
    public static final int RELAY_RINGLIGHT_POWER           = 0;

    //
    // Solenoid channels.
    //

    //
    // Miscellaneous sensors and devices.
    //

    //
    // DriveBase subsystem.
    //
    public static final double ENCODER_X_INCHES_PER_COUNT   = 0.01321074310684615838703943056885;
    public static final double ENCODER_X_KP                 = 0.045;
    public static final double ENCODER_X_KI                 = 0.0;
    public static final double ENCODER_X_KD                 = 0.0;
    public static final double ENCODER_X_KF                 = 0.0;
    public static final double ENCODER_X_TOLERANCE          = 1.0;
    public static final double ENCODER_X_SETTLING           = 0.2;

    public static final double ENCODER_Y_INCHES_PER_COUNT   = 0.01621544056690844936881352331858;
    public static final double ENCODER_Y_KP                 = 0.025;
    public static final double ENCODER_Y_KI                 = 0.0;
    public static final double ENCODER_Y_KD                 = 0.01;
    public static final double ENCODER_Y_KF                 = 0.0;
    public static final double ENCODER_Y_TOLERANCE          = 1.0;
    public static final double ENCODER_Y_SETTLING           = 0.2;

    public static final double GYRO_TURN_KP                 = 0.02;//0.010;
    public static final double GYRO_TURN_KI                 = 0.0;
    public static final double GYRO_TURN_KD                 = 0.0;
    public static final double GYRO_TURN_KF                 = 0.0;
    public static final double GYRO_TURN_TOLERANCE          = 1.0;
    public static final double GYRO_TURN_SETTLING           = 0.2;

    public static final double DRIVE_SLOW_XSCALE            = 3.0;
    public static final double DRIVE_SLOW_YSCALE            = 3.0;
    public static final double DRIVE_SLOW_TURNSCALE         = 3.0;

}   //class RobotInfo
