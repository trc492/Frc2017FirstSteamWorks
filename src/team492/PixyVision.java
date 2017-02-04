package team492;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;

public class PixyVision {

	private static final double Range = 1.65;
	
	public AnalogInput pixyCamera = null;
    public DigitalInput objectDetected = null;
    
    public PixyVision(final String instanceName)
    {
    	pixyCamera = new AnalogInput(1);	
	    objectDetected = new DigitalInput(9);
    }
    
    public boolean isTargetDetected(){
    	return objectDetected.get();
    }
    
    public double getTargetPosition(){
    	double voltage = pixyCamera.getVoltage();
    	return (voltage - Range)/Range;
    }
}
