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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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

import frclib.FrcPneumatic;

public class GearPickup
{
    private FrcPneumatic claw, arm;

    /**
     * Initialize the GearPickup class.
     * 
     * @param claw
     *            FrcPneumatic controlling the claw on this subsystem.
     * @param arm
     *            FrcPneumatic controlling the arm on this subsystem.
     */
    public GearPickup()
    {
        
        this.claw = new FrcPneumatic("GearPickupClaw",RobotInfo.CANID_PCM1,RobotInfo.SOL_GEARPICKUP_CLAW_EXTEND,RobotInfo.SOL_GEARPICKUP_CLAW_RETRACT);
        this.arm = new FrcPneumatic("GearPickupArm",RobotInfo.CANID_PCM1,RobotInfo.SOL_GEARPICKUP_ARM_EXTEND,RobotInfo.SOL_GEARPICKUP_ARM_RETRACT);
    }

    /**
     * Open the claw
     */
    public void openClaw()
    {
        claw.extend();
    }

    /**
     * Close the claw
     */
    public void closeClaw()
    {
        claw.retract();
    }

    /**
     * Get whether the claw is opened or not
     * 
     * @return true if opened, false if not
     */
    public boolean isClawOpen()
    {
        return claw.isExtended();
    }

    /**
     * 
     * @param up
     *            If true, open the claw. If false, close it.
     */
    public void setClawOpen(boolean open)
    {
        if (open)
            openClaw();
        else
            closeClaw();
    }

    /**
     * Lift the arm so the gear is perpendicular to the ground.
     */
    public void liftArm()
    {
        arm.retract();
    }

    /**
     * Lower the arm so the claw is on the ground.
     */
    public void lowerArm()
    {
        arm.extend();
    }

    /**
     * 
     * @param up
     *            If true, lift the arm. Otherwise, lower.
     */
    public void setArmUp(boolean up)
    {
        if (up)
            arm.retract();
        else
            arm.extend();
    }

    /**
     * Is the arm up or down?
     * 
     * @return Returns true if the arm is up, (gear is off ground) and returns
     *         false if the arm is down. (gear is on ground)
     */
    public boolean isArmUp()
    {
        return !arm.isExtended();
    }
}
