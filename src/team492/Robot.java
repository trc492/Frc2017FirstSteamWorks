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

import org.opencv.core.Rect;

import com.ctre.CANTalon.FeedbackDevice;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import frclib.FrcAHRSGyro;
import frclib.FrcCANTalon;
import frclib.FrcChoiceMenu;
import frclib.FrcFaceDetector;
import frclib.FrcGyro;
import frclib.FrcPneumatic;
import frclib.FrcRobotBase;
import hallib.HalDashboard;
import trclib.TrcDbgTrace;
import trclib.TrcDriveBase;
import trclib.TrcGyro;
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

    public static final boolean USE_NAV_X = true;
    public static final boolean USE_ANALOG_GYRO = false;
    public static final boolean USE_GRIP_VISION = false;
    public static final boolean USE_AXIS_CAMERA = false;
    public static final boolean USE_FACE_DETECTOR = false;
    public static final boolean USE_FRONT_PIXY = true;
    public static final boolean USE_FRONT_PIXY_UART = true;
    public static final boolean USE_REAR_PIXY = false;
//    public static final boolean USE_PIXY_TEST = true;

    private static final boolean DEBUG_DRIVE_BASE = false;
    private static final boolean DEBUG_PID_DRIVE = false;
    private static final boolean DEBUG_GRIP_VISION = false;
    private static final boolean DEBUG_FACE_DETECTION = false;
    private static final boolean DEBUG_PIXY_VISION = false;
    private static final double DASHBOARD_UPDATE_INTERVAL = 0.1;

    public static enum MatchType
    {
        PRACTICE,
        QUALIFICATION,
        QUATER_FINAL,
        SEMI_FINAL,
        FINAL
    }   //enum MatchType

    public static enum Alliance
    {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance

    public HalDashboard dashboard = HalDashboard.getInstance();
    public TrcDbgTrace tracer = FrcRobotBase.getGlobalTracer();

    public double targetHeading = 0.0;

    private double nextUpdateTime = TrcUtil.getCurrentTime();

    //
    // Sensors.
    //
    public TrcGyro gyro = null;
    public AnalogInput pressureSensor = null;
    public AnalogInput ultrasonicSensor = null;

    //
    // VisionTarget subsystem.
    //
    public GripVision gripVision = null;
    public FrcFaceDetector faceDetector = null;
    public PixyVision frontPixy = null;
    public PixyVision rearPixy = null;
//    public PixyTest testPixy = null;

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
    public Relay ringLightsPower;
    public Relay flashLightsPower;
    public FrcPneumatic mailbox;
    public GearPickup gearPickup;
    public Winch winch;

    //
    // Menus.
    //
    public FrcChoiceMenu<MatchType> matchTypeMenu;
    public FrcChoiceMenu<Alliance> allianceMenu;

    public MatchType matchType;
    public int matchNumber;
    public Alliance alliance;
    public double driveTime;
    public double drivePower;
    public double driveDistance;
    public double drivePowerLimit;
    public double turnDegrees;

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
    public void robotInit()
    {
        //
        // Sensors.
        //
        if (USE_NAV_X)
        {
            gyro = new FrcAHRSGyro("NavX", SPI.Port.kMXP);
        }
        else if (USE_ANALOG_GYRO)
        {
            gyro = new FrcGyro("AnalogGyro", new AnalogGyro(RobotInfo.AIN_ANALOG_GYRO));
        }
        else
        {
            gyro = new FrcGyro("ADXRS450", new ADXRS450_Gyro());
        }
        pressureSensor = new AnalogInput(RobotInfo.AIN_PRESSURE_SENSOR);
        ultrasonicSensor = new AnalogInput(RobotInfo.AIN_ULTRASONIC_SENSOR);

        //
        // VisionTarget subsystem.
        //
        if (USE_GRIP_VISION)
        {
            CvSink videoIn;
            CvSource videoOut;

            if (USE_AXIS_CAMERA)
            {
                AxisCamera axisCam = CameraServer.getInstance().addAxisCamera("axis-camera.local");
                axisCam.setResolution(RobotInfo.CAM_WIDTH, RobotInfo.CAM_HEIGHT);
                axisCam.setFPS(RobotInfo.CAM_FRAME_RATE);
                axisCam.setBrightness(RobotInfo.CAM_BRIGHTNESS);
                videoIn = CameraServer.getInstance().getVideo(axisCam);
            }
            else
            {
                //
                // Use USB camera.
                //
                UsbCamera cam0 = CameraServer.getInstance().startAutomaticCapture("cam0", 0);
                cam0.setResolution(RobotInfo.CAM_WIDTH, RobotInfo.CAM_HEIGHT);
                cam0.setFPS(RobotInfo.CAM_FRAME_RATE);
                cam0.setBrightness(RobotInfo.CAM_BRIGHTNESS);
                videoIn = CameraServer.getInstance().getVideo(cam0);
            }
            videoOut = CameraServer.getInstance().putVideo("VisionTarget", RobotInfo.CAM_WIDTH, RobotInfo.CAM_HEIGHT);

            gripVision = new GripVision("GripVision", videoIn, videoOut);
        }
        else if (USE_FACE_DETECTOR)
        {
            UsbCamera cam0 = CameraServer.getInstance().startAutomaticCapture("cam0", 0);
            cam0.setResolution(RobotInfo.CAM_WIDTH, RobotInfo.CAM_HEIGHT);
            cam0.setFPS(RobotInfo.CAM_FRAME_RATE);
            CvSink videoIn = CameraServer.getInstance().getVideo(cam0);
            CvSource videoOut =
                CameraServer.getInstance().putVideo("FaceDetector", RobotInfo.CAM_WIDTH, RobotInfo.CAM_HEIGHT);

            faceDetector = new FrcFaceDetector(
                "FaceDetector", "/home/lvuser/cascade-files/haarcascade_frontalface_alt.xml", videoIn, videoOut);
        }
//      else if (USE_PIXY_TEST)
//      {
//          testPixy = new PixyTest(I2C.Port.kOnboard, RobotInfo.PIXYCAM_FRONT_I2C_ADDRESS);
//      }
        else
        {
            if (USE_FRONT_PIXY)
            {
                if (USE_FRONT_PIXY_UART)
                {
                    frontPixy = new PixyVision(
                        "FrontPixy", RobotInfo.PIXY_LIFT_SIGNATURE, RobotInfo.PIXY_FRONT_BRIGHTNESS,
                        RobotInfo.PIXY_FRONT_ORIENTATION, SerialPort.Port.kMXP);
                }
                else
                {
                    frontPixy = new PixyVision(
                        "FrontPixy", RobotInfo.PIXY_LIFT_SIGNATURE, RobotInfo.PIXY_FRONT_BRIGHTNESS,
                        RobotInfo.PIXY_FRONT_ORIENTATION, I2C.Port.kMXP, RobotInfo.PIXYCAM_FRONT_I2C_ADDRESS);
                }
            }

            if (USE_REAR_PIXY)
            {
                rearPixy = new PixyVision(
                    "RearPixy", RobotInfo.PIXY_GEAR_SIGNATURE, RobotInfo.PIXY_REAR_BRIGHTNESS,
                    RobotInfo.PIXY_REAR_ORIENTATION, I2C.Port.kMXP, RobotInfo.PIXYCAM_REAR_I2C_ADDRESS);
            }
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

        leftFrontWheel.setPositionSensorInverted(false);
        leftRearWheel.setPositionSensorInverted(false);
        rightFrontWheel.setPositionSensorInverted(true);
        rightRearWheel.setPositionSensorInverted(true);

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
        ringLightsPower = new Relay(RobotInfo.RELAY_RINGLIGHT_POWER);
        ringLightsPower.setDirection(Direction.kForward);
        flashLightsPower = new Relay(RobotInfo.RELAY_FLASHLIGHT_POWER);
        flashLightsPower.setDirection(Direction.kForward);
        mailbox = new FrcPneumatic(
            "Mailbox", RobotInfo.CANID_PCM1, RobotInfo.SOL_MAILBOX_EXTEND, RobotInfo.SOL_MAILBOX_RETRACT);
        gearPickup = new GearPickup();
        winch = new Winch();

        //
        // Create Global Menus (can be used in all modes).
        //
        matchTypeMenu = new FrcChoiceMenu<>("Match Type");
        allianceMenu = new FrcChoiceMenu<>("Alliance");

        //
        // Populate Global Menus.
        //
        matchTypeMenu.addChoice("Practice", MatchType.PRACTICE);
        matchTypeMenu.addChoice("Qualification", MatchType.QUALIFICATION);
        matchTypeMenu.addChoice("Quater-final", MatchType.QUATER_FINAL);
        matchTypeMenu.addChoice("Semi-final", MatchType.SEMI_FINAL);
        matchTypeMenu.addChoice("Final", MatchType.FINAL);

        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE);

        //
        // Retrieve Global Choices.
        //
        matchType = matchTypeMenu.getCurrentChoiceObject();
        matchNumber = (int)HalDashboard.getNumber("MatchNumber", 0.0);
        alliance = allianceMenu.getCurrentChoiceObject();
        driveTime = HalDashboard.getNumber("DriveTime", 5.0);
        drivePower = HalDashboard.getNumber("DrivePower", 0.2);
        driveDistance = HalDashboard.getNumber("DriveDistance", 12.0);
        drivePowerLimit = HalDashboard.getNumber("DrivePowerLimit", 0.5);
        turnDegrees = HalDashboard.getNumber("TurnDegrees", 90.0);

        //
        // Robot Modes.
        //
        teleOpMode = new FrcTeleOp(this);
        autoMode = new FrcAuto(this);
        testMode = new FrcTest(this);
        setupRobotModes(teleOpMode, autoMode, testMode, null);
    }   //robotInit

    public void robotStartMode()
    {
    }   //robotStartMode

    public void robotStopMode()
    {
        driveBase.stop();
    }   //robotStopMode

    public double getPressure()
    {
        return 50.0*pressureSensor.getVoltage() - 25.0;
    }   //getPressure

    public double getUltrasonicDistance()
    {
        return ultrasonicSensor.getVoltage()*1024.0/25.4; 
    }   //getUltrasonicDistance

    public void setVisionEnabled(boolean enabled)
    {
        if (gripVision != null)
        {
            ringLightsPower.set(enabled? Value.kOn: Value.kOff);
            gripVision.setVideoOutEnabled(enabled);
            gripVision.setEnabled(enabled);
            tracer.traceInfo("TeleOp", "Grip Vision is %s!", Boolean.toString(enabled));
        }
        else if (faceDetector != null)
        {
            faceDetector.setVideoOutEnabled(enabled);
            faceDetector.setEnabled(enabled);
            tracer.traceInfo("TeleOp", "Face Detector is %s!", Boolean.toString(enabled));
        }
        else
        {
            ringLightsPower.set(enabled? Value.kOn: Value.kOff);

            if (frontPixy != null)
            {
                frontPixy.setEnabled(enabled);
            }

            if (rearPixy != null)
            {
                rearPixy.setEnabled(enabled);
            }
        }
    }   //setVisionEnabled

    public void updateDashboard()
    {
        double currTime = TrcUtil.getCurrentTime();

        if (currTime >= nextUpdateTime)
        {
            nextUpdateTime = currTime + DASHBOARD_UPDATE_INTERVAL;

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

            if (DEBUG_GRIP_VISION)
            {
                if (gripVision != null && gripVision.isEnabled())
                {
                    Rect[] targetRects = gripVision.getObjectRects();
                    tracer.traceInfo("GripVision", "Target is %s (%d)",
                        targetRects == null? "not found": "found", targetRects == null? 0: targetRects.length);
                    if (targetRects != null)
                    {
                        for (int i = 0; i < targetRects.length; i++)
                        {
                            dashboard.displayPrintf(1 + i, "x=%d, y=%d, width=%d, height=%d",
                                targetRects[i].x, targetRects[i].y, targetRects[i].width, targetRects[i].height);
                            tracer.traceInfo("TargetRect", "%02d: x=%d, y=%d, width=%d, height=%d",
                                i, targetRects[i].x, targetRects[i].y, targetRects[i].width, targetRects[i].height);
                        }
                    }
                }
            }

            if (DEBUG_FACE_DETECTION)
            {
                if (faceDetector != null && faceDetector.isEnabled())
                {
                    Rect[] faceRects = faceDetector.getFaceRects();
                    if (faceRects != null)
                    {
                        for (int i = 0; i < faceRects.length; i++)
                        {
                            dashboard.displayPrintf(1 + i, "x=%d, y=%d, width=%d, height=%d",
                                faceRects[i].x, faceRects[i].y, faceRects[i].width, faceRects[i].height);
                            tracer.traceInfo("FaceRect", "%02d: x=%d, y=%d, width=%d, height=%d",
                                i, faceRects[i].x, faceRects[i].y, faceRects[i].width, faceRects[i].height);
                        }
                    }
                }
            }

            if (frontPixy != null && frontPixy.isEnabled())
            {
                PixyVision.TargetInfo targetInfo = frontPixy.getTargetInfo();
                if (targetInfo != null)
                {
                    dashboard.displayPrintf(1, "x=%d, y=%d, width=%d, height=%d",
                        targetInfo.rect.x, targetInfo.rect.y, targetInfo.rect.width, targetInfo.rect.height);
                    dashboard.displayPrintf(2, "distance=%.1f, angle=%.1f", targetInfo.distance, targetInfo.angle);
                    if (DEBUG_PIXY_VISION)
                    {
                        tracer.traceInfo("PixyVision", "x=%d, y=%d, width=%d, height=%d, distance=%.1f, angle=%.1f",
                            targetInfo.rect.x, targetInfo.rect.y, targetInfo.rect.width, targetInfo.rect.height,
                            targetInfo.distance, targetInfo.angle);
                    }
                }
            }
        }
    }   //updateDashboard

    public void startTraceLog()
    {
        String filePrefix = matchType.toString();
        if (matchType != MatchType.PRACTICE) filePrefix += matchNumber;
        tracer.openTraceLog("/home/lvuser/tracelog", filePrefix);
    }   //startTraceLog

    public void stopTraceLog()
    {
        tracer.closeTraceLog();
    }   //stopTraceLog

    public void traceStateInfo(double elapsedTime, String stateName)
    {
        tracer.traceInfo(moduleName, "[%5.3f] %10s: xPos=%6.2f,yPos=%6.2f,heading=%6.1f/%6.1f",
            elapsedTime, stateName, driveBase.getXPosition(), driveBase.getYPosition(), driveBase.getHeading(),
            targetHeading);
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
        else if (pidCtrl == gyroTurnPidCtrl)
        {
            value = driveBase.getHeading();
        }

        return value;
    }   //getInput

}   //class Robot
