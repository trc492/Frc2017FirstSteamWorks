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

package team492;

import frclib.FrcPixyCam;

public class RobotInfo
{
    //
    // Compiler switches
    //

    //
    // Robot Dimensions.
    //
    public static final double ROBOT_LENGTH                     = 40.0;
    public static final double ROBOT_WIDTH                      = 36.0;
    public static final double ROBOT_HEIGHT                     = 24.0;

    //
    // Joystick ports.
    //
    public static final int JSPORT_LEFT_DRIVESTICK              = 0;
    public static final int JSPORT_RIGHT_DRIVESTICK             = 1;
    public static final int JSPORT_OPERATORSTICK                = 2;

    //
    // Analog Input ports.
    //
    public static final int AIN_PRESSURE_SENSOR                 = 0;
    public static final int AIN_ANALOG_GYRO                     = 1;
    public static final int AIN_PIXYCAM_OBJECT_POS              = 2;
    //
    // Digital Input ports.
    //
    public static final int DIN_GEAR_SENSOR                     = 0;
    public static final int DIN_PROXIMITY_SENSOR                = 1;
    public static final int DIN_PIXYCAM_OBJECTS_DETECTED        = 9;

    //
    // CAN IDs.
    //
    public static final int CANID_LEFTFRONTWHEEL                = 3;    // 40A: Orange
    public static final int CANID_RIGHTFRONTWHEEL               = 4;    // 40A: Yellow
    public static final int CANID_LEFTREARWHEEL                 = 5;    // 40A: Green
    public static final int CANID_RIGHTREARWHEEL                = 6;    // 40A: Blue
    public static final int CANID_SPARE                         = 7;    // 40A: Purple
    public static final int CANID_WINCH                         = 8;    // 40A: Gray

    public static final int CANID_PDP                           = 16;
    public static final int CANID_PCM1                          = 17;
    public static final int CANID_PCM2                          = 18;

    //
    // Relay channels.
    //
    public static final int RELAY_RINGLIGHT_POWER               = 0;
    public static final int RELAY_FLASHLIGHT_POWER              = 1;

    //
    // Solenoid channels.
    //
    public static final int SOL_GEARPICKUP_CLAW_RETRACT         = 0;
    public static final int SOL_GEARPICKUP_CLAW_EXTEND          = 1;
    public static final int SOL_GEARPICKUP_ARM_RETRACT          = 2;
    public static final int SOL_GEARPICKUP_ARM_EXTEND           = 3;
    public static final int SOL_MAILBOX_RETRACT                 = 4;
    public static final int SOL_MAILBOX_EXTEND                  = 5;

    //
    // Miscellaneous sensors and devices.
    //
    public static final int CAM_WIDTH                           = 320;
    public static final int CAM_HEIGHT                          = 240;
    public static final int CAM_FRAME_RATE                      = 15;
    public static final int CAM_BRIGHTNESS                      = 20;

    //
    // DriveBase subsystem.
    //

    // 20-21-2017: 0.0091442577063687, 0.17, 0.0, 0.0
    public static final double ENCODER_X_INCHES_PER_COUNT       = 0.0091442577063687;
    public static final double ENCODER_X_KP                     = 0.17;
    public static final double ENCODER_X_KI                     = 0.0;
    public static final double ENCODER_X_KD                     = 0.0;
    public static final double ENCODER_X_KF                     = 0.0;
    public static final double ENCODER_X_TOLERANCE              = 2.0;
    public static final double ENCODER_X_SETTLING               = 0.2;

    // 02-21-2017: 0.0159419007257628, 0.03, 0.0, 0.0075
    public static final double ENCODER_Y_INCHES_PER_COUNT       = 0.0159419007257628;
    public static final double ENCODER_Y_KP                     = 0.03;
    public static final double ENCODER_Y_KI                     = 0.0;
    public static final double ENCODER_Y_KD                     = 0.007;
    public static final double ENCODER_Y_KF                     = 0.0;
    public static final double ENCODER_Y_TOLERANCE              = 2.0;
    public static final double ENCODER_Y_SETTLING               = 0.2;

    public static final double GYRO_TURN_KP                     = 0.018;
    public static final double GYRO_TURN_KI                     = 0.0;
    public static final double GYRO_TURN_KD                     = 0.0;
    public static final double GYRO_TURN_KF                     = 0.0;
    public static final double GYRO_TURN_TOLERANCE              = 2.0;
    public static final double GYRO_TURN_SETTLING               = 0.2;

    public static final double DRIVE_SLOW_XSCALE                = 3.0;
    public static final double DRIVE_SLOW_YSCALE                = 3.0;
    public static final double DRIVE_SLOW_TURNSCALE             = 3.0;

    //
    // Vision subsystem.
    //
    public static final int PIXYCAM_FRONT_I2C_ADDRESS           = FrcPixyCam.DEF_I2C_ADDRESS;
    public static final int PIXYCAM_REAR_I2C_ADDRESS            = PIXYCAM_FRONT_I2C_ADDRESS + 2;
    public static final int PIXYCAM_WIDTH                       = 320;
    public static final int PIXYCAM_HEIGHT                      = 200;
    public static final int PIXY_TARGET_SIGNATURE               = 1;
    public static final int PIXY_BRIGHTNESS                     = 25;
    public static final PixyVision.Orientation PIXY_ORIENTATION = PixyVision.Orientation.CLOCKWISE_PORTRAIT;
    public static final double PIXYCAM_MID_VOLT                 = 3.3/2.0;  // in volts

    //
    // Winch subsystem.
    //
    public static final double WINCH_POWER_SCALE                = 3.0;
    public static final double WINCH_POSITION_SCALE             = 1.0;

}   // class RobotInfo
