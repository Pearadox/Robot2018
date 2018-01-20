package org.usfirst.frc.team5414.robot.subsystems;



import org.usfirst.frc.team5414.robot.RobotMap;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class IMU extends Subsystem{

	AHRS ahrs; //navx
//	AnalogGyro gy = new AnalogGyro(RobotMap.GyroPort);
	
	public static double yawOffset;
	
	public IMU() {
		ahrs = new AHRS(SPI.Port.kMXP);
	}
	
    public void initDefaultCommand() {
    }
    
    public void initialize()
    {
    	ahrs.reset();
//    	gy.initGyro();
//    	gy.setSensitivity(.007);
//    	gy.reset();
    }
    
    public double getTrueYaw()
    {
    	return ahrs.getYaw();
//    	return gy.getAngle();
    }
    
    public double getYaw()
    {
    	return getTrueYaw() + yawOffset;
    }
    
    public void zeroYaw()
    {
    	yawOffset += getYaw() - yawOffset;
    }
    
    public void reset()
    {
    	yawOffset = 0;
    	ahrs.reset();
    }
}