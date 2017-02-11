package team492;

import team492.CmdPidDrive.State;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdMidGearLift implements TrcRobot.RobotCommand{
	private enum State
    {
        DO_DELAY,
		MOVE_FORWARD1,
        VISION_GEAR_DEPLOY,
        MOVE_SIDEWAYS,
        MOVE_FORWARD2, 
        DONE
    }   //enum State
	
	private static final String moduleName = "CmdMidGearLift";

    private Robot robot;
    private double delay;
    private double forward1Distance;
    private double forward2Distance;
    private double sidewaysDistance;
    private double heading;
    private TrcEvent event;
    private TrcTimer timer;
    private TrcStateMachine<State> sm;
    private CmdVisionGearDeploy cmdVisionDeploy;
	
    CmdMidGearLift(Robot robot, double delay, double xDistance, double yDistance, double heading)
    {
        this.robot = robot;
        this.delay = delay;
        this.forward1Distance = forward1Distance;
        this.forward2Distance = forward2Distance;
        this.sidewaysDistance = sidewaysDistance;
        this.heading = 0.0;
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DO_DELAY);
        cmdVisionDeploy = new CmdVisionGearDeploy (robot);
    }   //CmdMidGearLift
    
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        boolean done = !sm.isEnabled();
        //
        // Print debug info.
        //
        State state = sm.getState();
        robot.dashboard.displayPrintf(1, "State: %s", state != null? sm.getState().toString(): "Disabled");

        if (sm.isReady())
        {
            state = sm.getState();

            switch (state)
            {
                case DO_DELAY:
                    //
                    // Do delay if any.
                    //
                    if (delay == 0.0)
                    {
                        sm.setState(State.MOVE_FORWARD1);
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.waitForSingleEvent(event, State.MOVE_FORWARD1);
                    }
                    break;

                case MOVE_FORWARD1:
                    //
                    // Drive the set distance and heading.
                    //
                    robot.pidDrive.setTarget(0.0, forward1Distance, heading, false, event);
                    sm.waitForSingleEvent(event, State.VISION_GEAR_DEPLOY);
                    break;
                    
                case VISION_GEAR_DEPLOY:
                	if (cmdVisionDeploy.cmdPeriodic(elapsedTime)){
                		sm.setState(State.MOVE_SIDEWAYS);
                	}
                	
                	break;
            	
                case MOVE_SIDEWAYS:
                	
                	robot.pidDrive.setTarget(sidewaysDistance, 0.0, heading, false, event);
                    sm.waitForSingleEvent(event, State.MOVE_FORWARD2);
                    break;
                
                case MOVE_FORWARD2:
                	
                	robot.pidDrive.setTarget(0.0, forward2Distance, heading, false, event);
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

        }
	
    }
    
}
