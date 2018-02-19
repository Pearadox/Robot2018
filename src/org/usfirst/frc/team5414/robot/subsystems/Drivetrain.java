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
    private TalonSRX right_motor2talon, left_motor2talon;
	public SpeedController right1, right2, left1, left2;
    public SpeedControllerGroup right;
    public SpeedControllerGroup left;
    private DifferentialDrive drive;
    private static Encoder encoderL, encoderR;
    
  private static final double kThrottleDeadband = 0.02;
  private static final double kWheelDeadband = 0.02;

  // These factor determine how fast the wheel traverses the "non linear" sine curve.
  private static final double kHighWheelNonLinearity = 0.65;
  private static final double kLowWheelNonLinearity = 0.5;

  private static final double kHighNegInertiaScalar = 4.0;

  private static final double kLowNegInertiaThreshold = 0.65;
  private static final double kLowNegInertiaTurnScalar = 3.5;
  private static final double kLowNegInertiaCloseScalar = 4.0;
  private static final double kLowNegInertiaFarScalar = 5.0;

  private static final double kHighSensitivity = 0.95;
  private static final double kLowSensitiity = 1.3;

  private static final double kQuickStopDeadband = 0.2;
  private static final double kQuickStopWeight = 0.1;
  private static final double kQuickStopScalar = 5.0;
  
  private double mOldWheel = 0.0;
  private double mQuickStopAccumlator = 0.0;
  private double mNegInertiaAccumlator = 0.0;

    public Drivetrain(){
    	if(RobotMap.flatbot) {
//	    	encoderR = new Encoder(RobotMap.DIOencoderRa, RobotMap.DIOencoderRb, false, Encoder.EncodingType.k4X);
//	    	encoderL = new Encoder(RobotMap.DIOencoderLa, RobotMap.DIOencoderLb, false, Encoder.EncodingType.k4X);
//	    	encoderR.reset();
//	    	encoderL.reset();
//	    	encoderR.setDistancePerPulse(RobotMap.LengthPerTickFeetFlat);
//	    	encoderL.setDistancePerPulse(RobotMap.LengthPerTickFeetFlat);
//	    	encoderR.setReverseDirection(true);
//	    	encoderL.setReverseDirection(true);
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
	    	drive = new DifferentialDrive(right, left);
    	}
    	else if (!RobotMap.flatbot && !RobotMap.compbot){
    		encoderR = new Encoder(RobotMap.DIOencoderRa, RobotMap.DIOencoderRb, false, Encoder.EncodingType.k4X);
	    	encoderL = new Encoder(RobotMap.DIOencoderLa, RobotMap.DIOencoderLb, false, Encoder.EncodingType.k4X);
	    	encoderR.reset();
	    	encoderL.reset();
	    	encoderR.setDistancePerPulse(RobotMap.LengthPerTickFeetPly);
	    	encoderL.setDistancePerPulse(RobotMap.LengthPerTickFeetPly);
	    	encoderR.setReverseDirection(true);
	    	encoderL.setReverseDirection(true);
	    	
	    	left_motor1 = new WPI_VictorSPX(RobotMap.CANLeftMotor1);
	    	left_motor2 = new WPI_VictorSPX(RobotMap.CANLeftMotor2);
	    	left_motor3 = new WPI_VictorSPX(RobotMap.CANLeftMotor3);
			right_motor1 = new WPI_VictorSPX(RobotMap.CANRightMotor1);
			right_motor2 = new WPI_VictorSPX(RobotMap.CANRightMotor2);
			right_motor3 = new WPI_VictorSPX(RobotMap.CANRightMotor3);
			left = new SpeedControllerGroup(left_motor1, left_motor2, left_motor3);
			right = new SpeedControllerGroup(right_motor1, right_motor2, right_motor3);
			drive = new DifferentialDrive(left, right);

			
			left_motor2.setInverted(true);
			right_motor2.setInverted(true);
    	}
    	else if(RobotMap.compbot)
    	{
	    	left_motor1 = new WPI_VictorSPX(RobotMap.CANLeftMotor1);
	    	left_motor2talon = new TalonSRX(RobotMap.CANLeftMotor2);
	    	left_motor3 = new WPI_VictorSPX(RobotMap.CANLeftMotor3);
			right_motor1 = new WPI_VictorSPX(RobotMap.CANRightMotor1);
			right_motor2talon = new TalonSRX(RobotMap.CANRightMotor2);
			right_motor3 = new WPI_VictorSPX(RobotMap.CANRightMotor3);
    	}
    }
    
    public void cheesyDrive(double throttle, double wheel, boolean isQuickTurn,
    		boolean isHighGear) {
		
		wheel = handleDeadband(wheel, kWheelDeadband);
		throttle = handleDeadband(throttle, kThrottleDeadband);
		
		if(Math.abs(throttle) < .1)
		{
			wheel *= .85;
			Robot.drivetrain.drive(wheel, -wheel);
			return;
		}
		
		double negInertia = wheel - mOldWheel;
		mOldWheel = wheel;
		
		double wheelNonLinearity;
		if (isHighGear) {
		  wheelNonLinearity = kHighWheelNonLinearity;
		  final double denominator = Math.sin(Math.PI / 2.0 * wheelNonLinearity);
		  // Apply a sin fusnction that's scaled to make it feel better.
		  wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
		  wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
		} else {
		  wheelNonLinearity = kLowWheelNonLinearity;
		  final double denominator = Math.sin(Math.PI / 2.0 * wheelNonLinearity);
		  // Apply a sin function that's scaled to make it feel better.
		  wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
		  wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
		  wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
		}
		
		double leftPwm, rightPwm, overPower;
		double sensitivity;
		
		double angularPower;
		double linearPower;
		
		// Negative inertia!
		double negInertiaScalar;
		if (isHighGear) {
		  negInertiaScalar = kHighNegInertiaScalar;
		  sensitivity = kHighSensitivity;
		} else {
		  if (wheel * negInertia > 0) {
		      // If we are moving away from 0.0, aka, trying to get more wheel.
		      negInertiaScalar = kLowNegInertiaTurnScalar;
		  } else {
		      // Otherwise, we are attempting to go back to 0.0.
		      if (Math.abs(wheel) > kLowNegInertiaThreshold) {
		          negInertiaScalar = kLowNegInertiaFarScalar;
		      } else {
		          negInertiaScalar = kLowNegInertiaCloseScalar;
		      }
		  }
		  sensitivity = kLowSensitiity;
		}
		double negInertiaPower = negInertia * negInertiaScalar;
		mNegInertiaAccumlator += negInertiaPower;
		
		wheel = wheel + mNegInertiaAccumlator;
		if (mNegInertiaAccumlator > 1) {
		  mNegInertiaAccumlator -= 1;
		} else if (mNegInertiaAccumlator < -1) {
		  mNegInertiaAccumlator += 1;
		} else {
		  mNegInertiaAccumlator = 0;
		}
		linearPower = throttle;
		
		// Quickturn!
		if (isQuickTurn) {
		  if (Math.abs(linearPower) < kQuickStopDeadband) {
		      double alpha = kQuickStopWeight;
//		      mQuickStopAccumlator = (1 - alpha) * mQuickStopAccumlator
//		              + alpha * Util.limit(wheel, 1.0) * kQuickStopScalar;
		  }
		  overPower = 1.0;
		  angularPower = wheel;
		} else {
		  overPower = 0.0;
		  angularPower = Math.abs(throttle) * wheel * sensitivity - mQuickStopAccumlator;
		  if (mQuickStopAccumlator > 1) {
		      mQuickStopAccumlator -= 1;
		  } else if (mQuickStopAccumlator < -1) {
		      mQuickStopAccumlator += 1;
		  } else {
		      mQuickStopAccumlator = 0.0;
		  }
		}
		
		rightPwm = leftPwm = linearPower;
		leftPwm += angularPower;
		rightPwm -= angularPower;
		
		if (leftPwm > 1.0) {
		  rightPwm -= overPower * (leftPwm - 1.0);
		  leftPwm = 1.0;
		} else if (rightPwm > 1.0) {
		  leftPwm -= overPower * (rightPwm - 1.0);
		  rightPwm = 1.0;
		} else if (leftPwm < -1.0) {
		  rightPwm += overPower * (-1.0 - leftPwm);
		  leftPwm = -1.0;
		} else if (rightPwm < -1.0) {
		  leftPwm += overPower * (-1.0 - rightPwm);
		  rightPwm = -1.0;
		}
		if(Robot.oi.getJoystick().getRawButtonPressed(11)) zeroEncoders();
    	if(Robot.oi.getJoystick().getRawButton(11)) 
    	{
    		System.out.print(getEncoderLFeet() + " " + getEncoderRFeet() + " ");
    	}
    	if(Robot.oi.getJoystick().getRawButtonReleased(11)) System.out.println("----------------------------");
		drive(leftPwm, rightPwm);
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
    	if(Robot.oi.getJoystick().getRawButtonPressed(11)) zeroEncoders();
    	if(Robot.oi.getJoystick().getRawButton(11)) 
    	{
//    		System.out.print(String.format("%.7s", ax2) + " " + String.format("%.7s", ax1) + " ");
    		System.out.print(getEncoderL() + " " + getEncoderR() + " ");
    	}
    	if(Robot.oi.getJoystick().getRawButtonReleased(11)) System.out.println("----------------------------");
    	if(RobotMap.flatbot) arcadeDrive(-ax1, ax2, true);
    	else arcadeDrive(-ax2, -ax1*.7, true);
		
    }
	
	public double handleDeadband(double val, double deadband) {
      return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
  }
    
    public void arcadeDrive(double throttle, double twist, boolean squared){
    	if(RobotMap.compbot)
    	{
    		if (squared) {
    			throttle = Math.copySign(throttle * throttle * throttle, throttle);
  		      	twist = -Math.copySign(twist * twist * twist, twist);
    		}
    		double leftSpeed = throttle - twist;
    		double rightSpeed = -throttle - twist;
    		right_motor1.set(rightSpeed);
    		right_motor2talon.set(ControlMode.PercentOutput, rightSpeed);
    		right_motor3.set(rightSpeed);
    		left_motor1.set(leftSpeed);
    		left_motor2talon.set(ControlMode.PercentOutput, leftSpeed);
    		left_motor3.set(leftSpeed);
    	}
    	else drive.arcadeDrive(throttle,twist, squared);
    }
    
    public void drive(double l, double r)
    {
    	if(RobotMap.compbot)
    	{
    		right_motor1.set(r);
    		right_motor2talon.set(ControlMode.PercentOutput, r);
    		right_motor3.set(r);
    		left_motor1.set(l);
    		left_motor2talon.set(ControlMode.PercentOutput, l);
    		left_motor3.set(l);
    	}
    	else if(RobotMap.flatbot) drive.tankDrive(-l, r);
    	else drive.tankDrive(l, r);
    }
    
    public double getEncoderRFeet()
    {
    	if(RobotMap.flatbot) return getEncoderR() * RobotMap.LengthPerTickFeetFlat;
    	else return getEncoderR() * RobotMap.LengthPerTickFeetPly;
    }
    
    public double getEncoderLFeet()
    {
    	if(RobotMap.flatbot) return getEncoderL() * RobotMap.LengthPerTickFeetFlat;
    	else return getEncoderL() * RobotMap.LengthPerTickFeetPly;
    }
    
    public int getEncoderR()
    {
    	try {
    		return encoderR.get();
    	} catch(Exception e) {}
    	return encoderR.get();
    }
    
    public int getEncoderL()
    {
    	try {
    		return encoderL.get();
    	} catch(Exception e) {}
    	return encoderL.get();
    }
    
    public void zeroEncoders()
    {
    	encoderR.reset();
    	encoderL.reset();
    }

	public void stop(){
    	drive(0,0);
    }
    
    public void initDefaultCommand() {
    	setDefaultCommand(new DrivewithJoystick());
    }
}

