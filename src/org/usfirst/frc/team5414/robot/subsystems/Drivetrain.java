package org.usfirst.frc.team5414.robot.subsystems;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;
import org.usfirst.frc.team5414.robot.commands.DrivewithJoystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
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

    private WPI_VictorSPX right_motor1, right_motor2, right_motor3, left_motor1, left_motor2, left_motor3;
	public SpeedController right1, right2, left1, left2;
    public SpeedControllerGroup right;
    public SpeedControllerGroup left;
    private DifferentialDrive drive;
    private static Encoder encoderFR, encoderFL, encoderBL, encoderBR; 
    private DoubleSolenoid LShift, RShift;

    public Drivetrain(){
    	if(RobotMap.flatbot) {
	    	encoderFR = new Encoder(RobotMap.DIOencoderFRa, RobotMap.DIOencoderFRb, false, Encoder.EncodingType.k4X);
	    	encoderFL = new Encoder(RobotMap.DIOencoderFLa, RobotMap.DIOencoderFLb, false, Encoder.EncodingType.k4X);
	    	encoderFR.reset();
	    	encoderFL.reset();
	    	encoderFR.setDistancePerPulse(RobotMap.CircumferenceMeters / 1440);
	    	encoderFL.setDistancePerPulse(RobotMap.CircumferenceMeters / 1440);
	    	encoderFR.setReverseDirection(true);
	    	encoderFL.setReverseDirection(true);
	    	right1 = new Victor(2);
	    	right2 = new Victor(3);
	    	left1 = new Victor(0);
	    	left2 = new Victor(1);
	    	right1.setInverted(true);
	    	right2.setInverted(true);
	    	right1.setInverted(true);
	    	right2.setInverted(true);
	    	left = new SpeedControllerGroup(left1, left2);
	    	right = new SpeedControllerGroup(right1, right2);
	    	drive = new DifferentialDrive(left, right);
    	}
    	else  {
	    	encoderFR = new Encoder(RobotMap.DIOencoderFRa, RobotMap.DIOencoderFRb, false, Encoder.EncodingType.k4X);
	    	encoderBR = new Encoder(RobotMap.DIOencoderBRa, RobotMap.DIOencoderBRb, false, Encoder.EncodingType.k4X);
	    	encoderBL = new Encoder(RobotMap.DIOencoderBLa, RobotMap.DIOencoderBLb, false, Encoder.EncodingType.k4X);
	    	encoderFL = new Encoder(RobotMap.DIOencoderFLa, RobotMap.DIOencoderFLb, false, Encoder.EncodingType.k4X);
	    	encoderFR.reset();
	    	encoderBR.reset();
	    	encoderBL.reset();
	    	encoderFL.reset();
	    	encoderFR.setDistancePerPulse(RobotMap.CircumferenceMeters / 1440);
	    	encoderFL.setDistancePerPulse(RobotMap.CircumferenceMeters / 1440);
	    	encoderBR.setDistancePerPulse(RobotMap.CircumferenceMeters / 1440);
	    	encoderBL.setDistancePerPulse(RobotMap.CircumferenceMeters / 1440);
	    	
	    	LShift = new DoubleSolenoid(RobotMap.LShiftA, RobotMap.LShiftB);
	    	RShift = new DoubleSolenoid(RobotMap.RShiftA, RobotMap.RShiftB);
	    	
	    	left_motor1 = new WPI_VictorSPX(RobotMap.CANLeftMotor1);
	    	left_motor2 = new WPI_VictorSPX(RobotMap.CANLeftMotor2);
	    	left_motor3 = new WPI_VictorSPX(RobotMap.CANLeftMotor3);
			right_motor1 = new WPI_VictorSPX(RobotMap.CANRightMotor1);
			right_motor2 = new WPI_VictorSPX(RobotMap.CANRightMotor2);
			right_motor3 = new WPI_VictorSPX(RobotMap.CANRightMotor3);
			left = new SpeedControllerGroup(left_motor1, left_motor2, left_motor3);
			right = new SpeedControllerGroup(right_motor1, right_motor2, right_motor3);
			drive = new DifferentialDrive(left_motor2, right_motor2);
			left_motor2.setInverted(true);
			right_motor2.setInverted(true);
    	}
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
    	ax1 *= -1;
    	if(Robot.oi.getJoystick().getRawButton(11)) 
    	{
//    		System.out.print(String.format("%.7s", ax2) + " " + String.format("%.7s", ax1) + " ");
    		System.out.print(getEncoderL() + " " + getEncoderR() + " ");
    	}
    	if(Robot.oi.getJoystick().getRawButtonReleased(11)) System.out.println();
    	if(RobotMap.flatbot) drive.arcadeDrive(ax1, ax2);
    	else drive.arcadeDrive(-ax1, ax2);
    }
    
    public void arcadeDrive(double throttle, double twist){
    	drive.arcadeDrive(throttle,twist);
    }
    
    public void drive(double l, double r)
    {
    	if(RobotMap.flatbot) drive.tankDrive(-l, r);
    	else drive.tankDrive(-l, -r);
    }
    
    public int getEncoderR()
    {
    	return getEncoderFL();
    }
    
    public int getEncoderL()
    {
    	return getEncoderFR();
    }
    
    public int getEncoderFR()
    {
    	return encoderFR.get();
    }
    
    public int getEncoderFL()
    {
    	return encoderFL.get();
    }
    
    public int getEncoderBL()
    {
    	return encoderBL.get();
    }
    
    public int getEncoderBR()
    {
    	return encoderBR.get();
    }
    
    public void zeroEncoders()
    {
    	encoderFR.reset();
    	encoderFL.reset();
    	if(!RobotMap.flatbot)
    	{
    		encoderBL.reset();
    		encoderBR.reset();
    	}
    	System.out.println(getEncoderFR());
    }

	public void stop(){
    	drive.tankDrive(0,0);
    }	
	
	public void shiftUp()
	{
		LShift.set(DoubleSolenoid.Value.kForward);
		RShift.set(DoubleSolenoid.Value.kForward);
	}
	
	public void shiftDown()
	{
		LShift.set(DoubleSolenoid.Value.kReverse);
		RShift.set(DoubleSolenoid.Value.kReverse);
	}
	
	public void shiftNone()
	{
		LShift.set(DoubleSolenoid.Value.kOff);
		RShift.set(DoubleSolenoid.Value.kOff);
	}
    
    public void initDefaultCommand() {
    	setDefaultCommand(new DrivewithJoystick());
    }
}

