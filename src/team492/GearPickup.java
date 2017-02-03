package team492;

import frclib.FrcPneumatic;

public class GearPickup {
	private FrcPneumatic claw, arm;
	/**
	 * Initialize the GearPickup class.
	 * @param claw FrcPneumatic controlling the claw on this subsystem.
	 * @param arm FrcPneumatic controlling the arm on this subsystem.
	 */
	public GearPickup(FrcPneumatic claw, FrcPneumatic arm){
		this.claw = claw;
		this.arm = arm;
	}
	
	/**
	 * Open the claw
	 */
	public void openClaw(){
		claw.extend();
	}
	
	/**
	 * Close the claw
	 */
	public void closeClaw(){
		claw.retract();
	}
	
	/**
	 * Get whether the claw is opened or not
	 * @return true if opened, false if not
	 */
	public boolean getClawOpen(){
		return claw.isExtended();
	}
	
	/**
	 * 
	 * @param pos If true, open the claw. If false, close it.
	 */
	public void setClawOpen(boolean pos){
		if(pos) openClaw();
		else closeClaw();
	}
	
	/**
	 * Lift the arm so the gear is perpendicular to the ground.
	 */
	public void liftArm(){
		arm.retract();
	}
	
	/**
	 * Lower the arm so the claw is on the ground.
	 */
	public void lowerArm(){
		arm.extend();
	}
	
	/**
	 * 
	 * @param pos If true, lift the arm. Otherwise, lower.
	 */
	public void setArmUp(boolean pos){
		if(pos)arm.retract();
		else arm.extend();
	}
	
	/**
	 * Is the arm up or down?
	 * @return Returns true if the arm is up, (gear is off ground) and returns false if the arm is down. (gear is on ground)
	 */
	public boolean getArmUp(){
		return !arm.isExtended();
	}
}
