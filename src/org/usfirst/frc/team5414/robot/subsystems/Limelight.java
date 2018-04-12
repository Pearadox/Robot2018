package org.usfirst.frc.team5414.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Limelight extends Subsystem {

	double x = 0;
	double y = 0;
	double area = 0;
	boolean hasTarget = false;
	double latency = -1;
	static boolean lightOn = true;
	
    public Limelight() {
		
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void update() {
    	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry tx = table.getEntry("tx");
		NetworkTableEntry ty = table.getEntry("ty");
		NetworkTableEntry tv = table.getEntry("tv");
		NetworkTableEntry tl = table.getEntry("tl");
		NetworkTableEntry ta = table.getEntry("ta");
    	x = tx.getDouble(0);
		y = ty.getDouble(0);
		hasTarget = (int)tv.getDouble(0) == 1;
		latency = tl.getDouble(-1);
		area = ta.getDouble(0);
    }
    
    public void lightOn() {
    	NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    	lightOn = true;
    }
    
    public void lightOff() {
    	NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    	lightOn = false;
    }
    
    public void toggleLight() {
    	if(lightOn)
    		lightOff();
    	else lightOn();
    }
    
    public void lightBlink() {
    	NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);
    }
    
    public void modeVision() {
    	NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    }
    
    public void modeCamera() {
    	NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    }
    
    public void setPipeline(int channel)
    {
    	NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(channel);
    }
    
    public double getX() {
    	update();
//    	return x;
    	return -y;
    }
    
    public double getY() {
    	update();
//    	return y;
    	return x;
    }
    
    public double getArea() {
    	update();
    	return area;
    }
    
    public boolean hasTarget() {
    	update();
    	return hasTarget;
    }
    
    public double getLatency() {
    	update();
    	return latency;
    }
}
