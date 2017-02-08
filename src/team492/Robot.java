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

import com.ctre.CANTalon.FeedbackDevice;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import frclib.FrcCANTalon;
import frclib.FrcFaceDetector;
import frclib.FrcGyro;
import frclib.FrcPneumatic;
import frclib.FrcRobotBase;
import hallib.HalDashboard;
import trclib.TrcDbgTrace;
import trclib.TrcDriveBase;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot.RobotMode;
import trclib.TrcUtil;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TrcRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the
 * resource directory.
 */
public class Robot extends FrcRobotBase implements TrcPidController.PidInput
{
    public static final String programName = "FirstSteamWorks";
    public static final String moduleName = "Robot";

    public static final boolean USE_VISION_TARGET = false;
    public static final boolean USE_FACE_DETECTOR = false;
    public static final boolean USE_PIXY_VISION = true;

    private static final boolean DEBUG_DRIVE_BASE = false;
    private static final boolean DEBUG_PID_DRIVE = false;
    private static final double DASHBOARD_UPDATE_INTERVAL = 0.1;

    public HalDashboard dashboard = HalDashboard.getInstance();
    public TrcDbgTrace tracer = FrcRobotBase.getGlobalTracer();
    public double targetHeading = 0.0;

    private double nextUpdateTime = TrcUtil.getCurrentTime();

    //
    // Sensors.
    //
    public FrcGyro gyro = null;

    //
    // VisionTarget subsystem.
    //
    public VisionTarget visionTarget = null;
    public FrcFaceDetector faceDetector = null;
    public PixyVision pixyVision = null;

    //
    // DriveBase subsystem.
    //
    public FrcCANTalon leftFrontWheel;
    public FrcCANTalon leftRearWheel;
    public FrcCANTalon rightFrontWheel;
    public FrcCANTalon rightRearWheel;
    public TrcDriveBase driveBase;

    public TrcPidController encoderXPidCtrl;
    public TrcPidController encoderYPidCtrl;
    public TrcPidController gyroTurnPidCtrl;
    public TrcPidDrive pidDrive;

    //
    // Define our subsystems for Auto and TeleOp modes.
    //
    public FrcPneumatic mailbox;
    public GearPickup gearPickup;

    public FrcCANTalon winch;   // MTS: Why? You already have a winch class?!
    //
    // Robot Modes.
    //
    private RobotMode teleOpMode;
    private RobotMode autoMode;
    private RobotMode testMode;

    /**
     * Constructor.
     */
    public Robot()
    {
        super(programName);
    }   //Robot

    /**
     * This function is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void initRobot()
    {
        //
        // Sensors.
        //
//        try
//        {
//            gyro = new FrcGyro("ADXRS450", new ADXRS450_Gyro());
//        }
//        catch (NullPointerException e)
//        {
//            gyro = null;
//        }

        if (USE_PIXY_VISION)
        {
            pixyVision = new PixyVision("PixyCamera");  //MTS: Don't really need instanceName.
        }

        //
        // VisionTarget subsystem.
        //
        if (USE_VISION_TARGET)
        {
            CameraServer.getInstance().startAutomaticCapture().setResolution(640, 480);
            visionTarget = new VisionTarget("GearTarget");
        }
        else if (USE_FACE_DETECTOR)
        {
            CameraServer.getInstance().startAutomaticCapture().setResolution(640, 480);
            faceDetector = new FrcFaceDetector(
                "FrontalFace", "/home/lvuser/cascade-files/haarcascade_frontalface_alt.xml");
        }

        //
        // DriveBase subsystem.
        //
        leftFrontWheel = new FrcCANTalon("LeftFrontWheel", RobotInfo.CANID_LEFTFRONTWHEEL);
        leftRearWheel = new FrcCANTalon("LeftRearWheel", RobotInfo.CANID_LEFTREARWHEEL);
        rightFrontWheel = new FrcCANTalon("RightFrontWheel", RobotInfo.CANID_RIGHTFRONTWHEEL);
        rightRearWheel = new FrcCANTalon("RightRearWheel", RobotInfo.CANID_RIGHTREARWHEEL);

        //
        // Initialize each drive motor controller.
        //
        leftFrontWheel.setInverted(false);
        leftRearWheel.setInverted(false);
        rightFrontWheel.setInverted(true);
        rightRearWheel.setInverted(true);

        leftFrontWheel.enableLimitSwitch(false, false);
        leftRearWheel.enableLimitSwitch(false, false);
        rightFrontWheel.enableLimitSwitch(false, false);
        rightRearWheel.enableLimitSwitch(false, false);

        leftFrontWheel.setPositionSensorInverted(true);
        leftRearWheel.setPositionSensorInverted(true);
        rightFrontWheel.setPositionSensorInverted(false);
        rightRearWheel.setPositionSensorInverted(false);

        leftFrontWheel.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        leftRearWheel.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        rightFrontWheel.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        rightRearWheel.setFeedbackDevice(FeedbackDevice.QuadEncoder);

        //
        // Initialize DriveBase subsystem.
        //
        driveBase = new TrcDriveBase(leftFrontWheel, leftRearWheel, rightFrontWheel, rightRearWheel, gyro);
        driveBase.setXPositionScale(RobotInfo.ENCODER_X_INCHES_PER_COUNT);
        driveBase.setYPositionScale(RobotInfo.ENCODER_Y_INCHES_PER_COUNT);

        //
        // Create PID controllers for DriveBase PID drive.
        //
        encoderXPidCtrl = new TrcPidController(
            "encoderXPidCtrl",
            RobotInfo.ENCODER_X_KP, RobotInfo.ENCODER_X_KI, RobotInfo.ENCODER_X_KD, RobotInfo.ENCODER_X_KF,
            RobotInfo.ENCODER_X_TOLERANCE, RobotInfo.ENCODER_X_SETTLING, this);
        encoderYPidCtrl = new TrcPidController(
            "encoderYPidCtrl",
            RobotInfo.ENCODER_Y_KP, RobotInfo.ENCODER_Y_KI, RobotInfo.ENCODER_Y_KD, RobotInfo.ENCODER_Y_KF,
            RobotInfo.ENCODER_Y_TOLERANCE, RobotInfo.ENCODER_Y_SETTLING, this);
        gyroTurnPidCtrl = new TrcPidController(
            "gyroTurnPidCtrl",
            RobotInfo.GYRO_TURN_KP, RobotInfo.GYRO_TURN_KI, RobotInfo.GYRO_TURN_KD, RobotInfo.GYRO_TURN_KF,
            RobotInfo.GYRO_TURN_TOLERANCE, RobotInfo.GYRO_TURN_SETTLING, this);
        gyroTurnPidCtrl.setAbsoluteSetPoint(true);
        pidDrive = new TrcPidDrive("pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroTurnPidCtrl);

        //
        // Create other subsystems.
        //
        mailbox = new FrcPneumatic(
            "Mailbox", RobotInfo.CANID_PCM1, RobotInfo.SOL_MAILBOX_EXTEND, RobotInfo.SOL_MAILBOX_RETRACT);
        gearPickup = new GearPickup();

        winch = new FrcCANTalon("Winch", RobotInfo.CANID_WINCH);    //MTS: You already have a winch class?!

        //
        // Robot Modes.
        //
        teleOpMode = new FrcTeleOp(this);
        autoMode = new FrcAuto(this);
        testMode = new FrcTest(this);
        setupRobotModes(teleOpMode, autoMode, testMode, null);
    }   //initRobot

    public void updateDashboard()
    {
        double currTime = TrcUtil.getCurrentTime();

        if (currTime >= nextUpdateTime)
        {
            nextUpdateTime = currTime + DASHBOARD_UPDATE_INTERVAL;

            //
            // Sensor info.
            //
            if (pixyVision != null && pixyVision.isTargetDetected())
            {
                dashboard.displayPrintf(1, "Pixy: %.3f", pixyVision.getTargetPosition());
//                double voltage = pixyCamera.getVoltage();
//                dashboard.displayPrintf(1, "Pixy: %.3f (%.3f)", (voltage - 1.65)/3.3, voltage);
            }

            if (DEBUG_DRIVE_BASE)
            {
                //
                // DriveBase debug info.
                //
                dashboard.displayPrintf(1, "DriveBase: lf=%.0f, rf=%.0f, lr=%.0f, rr=%.0f",
                    leftFrontWheel.getPosition(), rightFrontWheel.getPosition(),
                    leftRearWheel.getPosition(), rightRearWheel.getPosition());
                dashboard.displayPrintf(2, "DriveBase: X=%.1f, Y=%.1f, Heading=%.1f",
                    driveBase.getXPosition(), driveBase.getYPosition(), driveBase.getHeading());

                if (DEBUG_PID_DRIVE)
                {
                    encoderXPidCtrl.displayPidInfo(3);
                    encoderYPidCtrl.displayPidInfo(5);
                    gyroTurnPidCtrl.displayPidInfo(7);
                }
                HalDashboard.putNumber("DriveBase.X", driveBase.getXPosition());
                HalDashboard.putNumber("DriveBase.Y", driveBase.getYPosition());
                HalDashboard.putNumber("DriveBase.Heading", driveBase.getHeading());
            }
        }
    }   //updateDashboard

    public void traceStateInfo(double elapsedTime, String stateName)
    {
        tracer.traceInfo(
                moduleName, "[%5.3f] %10s: xPos=%6.2f,yPos=%6.2f,heading=%6.1f/%6.1f",
                elapsedTime, stateName,
                driveBase.getXPosition(), driveBase.getYPosition(), driveBase.getHeading(), targetHeading);
    }   //traceStateInfo

    //
    // Implements TrcPidController.PidInput.
    //
    @Override
    public double getInput(TrcPidController pidCtrl)
    {
        double value = 0.0;

        if (pidCtrl == encoderXPidCtrl)
        {
            value = driveBase.getXPosition();
        }
        else if (pidCtrl == encoderYPidCtrl)
        {
            value = driveBase.getYPosition();
        }
        else if (pidCtrl == gyroTurnPidCtrl && gyro != null)
        {
            value = gyro.getZHeading().value;
        }

        return value;
    }   //getInput

}   //class Robot
