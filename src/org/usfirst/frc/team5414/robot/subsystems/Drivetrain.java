package org.usfirst.frc.team5414.robot.subsystems;

import org.usfirst.frc.team5414.robot.RobotMap;
import org.usfirst.frc.team5414.robot.commands.DrivewithJoystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Drivetrain extends Subsystem {

    private SpeedController rightf_motor, rightb_motor, leftf_motor, leftb_motor;
    private SpeedControllerGroup right;
    private SpeedControllerGroup left;
    private DifferentialDrive drive;
    private Encoder encoderFR; 
    private Encoder encoderBR; 
    private Encoder encoderBL; 
    private Encoder encoderFL; 

    public Drivetrain(){
    	
    	encoderFR = new Encoder(RobotMap.DIOencoderFRa, RobotMap.DIOencoderFRb, false, Encoder.EncodingType.k4X);
    	encoderBR = new Encoder(RobotMap.DIOencoderBRa, RobotMap.DIOencoderBRb, false, Encoder.EncodingType.k4X);
    	encoderBL = new Encoder(RobotMap.DIOencoderBLa, RobotMap.DIOencoderBLb, false, Encoder.EncodingType.k4X);
    	encoderFL = new Encoder(RobotMap.DIOencoderFLa, RobotMap.DIOencoderFLb, false, Encoder.EncodingType.k4X);
    	encoderFR.reset();
    	encoderBR.reset();
    	encoderBL.reset();
    	encoderFL.reset();
    	
    	leftb_motor = new Victor(RobotMap.PWMLeftBackMotor);
		leftf_motor = new Victor(RobotMap.PWMLeftFrontMotor);
		rightb_motor = new Victor(RobotMap.PWMRightBackMotor);
		rightf_motor = new Victor(RobotMap.PWMRightFrontMotor);
		left = new SpeedControllerGroup(leftf_motor, leftb_motor);
		right = new SpeedControllerGroup(rightf_motor, rightb_motor);
		drive = new DifferentialDrive(left, right); 
    }
    
	public void arcadeDrive(Joystick stick){
    	double ax1; 	//X-axis of motion for robot
    	double ax2;		//Y-axis of motion for robot
    	
    	if(Math.abs(stick.getRawAxis(2)) < .087)	//Setting deadzone for the x-axis
    	{
    		ax1 = 0;
    	}
    	else
    	{
    		ax1 = stick.getRawAxis(2);
    	}
    	if(Math.abs(stick.getRawAxis(1)) < .18)		//Setting deazone for the y-axis
    	{
    		ax2 = 0;
    	}
    	else
    	{
    		ax2 = stick.getRawAxis(1);
    	}
    	
    	boolean ax1n = false, ax2n = false;
    	if(ax1 < 0) ax1n = true;
    	if(ax2 < 0) ax2n = true;
    	ax1 *= ax1; ax2 *= ax2; //scaling the output of the joystick to fine tune the end result
    	if(ax1n) ax1 *= -1;
    	if(ax2n) ax2 *= -1;
    	drive.arcadeDrive(ax2,-ax1);
    }
    
    public void arcadeDrive(double throttle, double twist){
    	drive.arcadeDrive(throttle,twist);
    }
    
    public void drive(double l, double r)
    {
    	drive.tankDrive(l, r);
    }
    
    public double getEncoderFR()
    {
    	return encoderFR.get();
    }
    
    public double getEncoderFL()
    {
    	return encoderFL.get();
    }
    
    public double getEncoderBL()
    {
    	return encoderBL.get();
    }
    
    public double getEncoderBR()
    {
    	return encoderBR.get();
    }
    
    public void zeroEncoders()
    {
    	encoderFR.reset();
    	encoderFL.reset();
    	encoderBL.reset();
    	encoderBR.reset();
    }
    
    public void tankDrive(double left, double right)
    {
    	drive.tankDrive(-left, -right);
    }
    
	public void stop(){
    	drive.tankDrive(0,0);
    }	
    
    public void initDefaultCommand() {
    	setDefaultCommand(new DrivewithJoystick());
    }
}

