package org.usfirst.frc.team5414.robot.subsystems;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;
import org.usfirst.frc.team5414.robot.Traj;
import org.usfirst.frc.team5414.robot.commands.DrivewithJoystick;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
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

    private WPI_VictorSPX rightSlave1, rightSlave2, leftSlave1, leftSlave2;
    private TalonSRX rightMaster, leftMaster;
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
  
  int encoderOffsetL = 0;
  int encoderOffsetR = 0;

    public Drivetrain(){
    	if(RobotMap.flatbot) {
	    	encoderR = new Encoder(RobotMap.DIOencoderRaFlat, RobotMap.DIOencoderRbFlat, false, Encoder.EncodingType.k4X);
	    	encoderL = new Encoder(RobotMap.DIOencoderLaFlat, RobotMap.DIOencoderLbFlat, false, Encoder.EncodingType.k4X);
	    	encoderR.reset();
	    	encoderL.reset();
	    	encoderR.setDistancePerPulse(RobotMap.FeetPerTickFlat);
	    	encoderL.setDistancePerPulse(RobotMap.FeetPerTickFlat);
	    	encoderR.setReverseDirection(true);
	    	encoderL.setReverseDirection(true);
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
    	else if(RobotMap.compbot)
    	{
    		encoderR = new Encoder(RobotMap.DIOencoderRbComp, RobotMap.DIOencoderRaComp, false, Encoder.EncodingType.k4X);
	    	encoderL = new Encoder(RobotMap.DIOencoderLaComp, RobotMap.DIOencoderLbComp, false, Encoder.EncodingType.k4X);
	    	leftSlave1 = new WPI_VictorSPX(RobotMap.CANLeftSlave1);
	    	leftMaster = new TalonSRX(RobotMap.CANLeftMaster);
	    	leftSlave2 = new WPI_VictorSPX(RobotMap.CANLeftSlave2);
			rightSlave1 = new WPI_VictorSPX(RobotMap.CANRightSlave1);
			rightMaster = new TalonSRX(RobotMap.CANRightMaster);
			rightSlave2 = new WPI_VictorSPX(RobotMap.CANRightSlave2);
			
			leftSlave1.follow(leftMaster);
	    	leftSlave2.follow(leftMaster);
	    	rightSlave1.follow(rightMaster);
	    	rightSlave2.follow(rightMaster);
	    	
			leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
			leftMaster.configNeutralDeadband(.01, 0);
			rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
			rightMaster.configNeutralDeadband(.01, 0);
			
			Thread updateEncodersOnTalons = new Thread() {
				public void run() {
					try {
						while(true) {
							leftMaster.setSelectedSensorPosition(getEncoderL(), 0, 0);
							rightMaster.setSelectedSensorPosition(getEncoderR(), 0, 0);
							Thread.sleep(2);
						}
					} catch(Exception e) {e.printStackTrace();}
				}
			};
			updateEncodersOnTalons.start();
			
    	}
    }
    
    public void setPIDLeft(double kF, double kP, double kI, double kD)
    {
    	RobotMap.MMLeftkD = kD;
    	RobotMap.MMLeftkF = kF;
    	RobotMap.MMLeftkI = kI;
    	RobotMap.MMLeftkP = kP;
    	leftMaster.config_kF(0, RobotMap.MMLeftkF, 10);
		leftMaster.config_kP(0, RobotMap.MMLeftkP, 10);
		leftMaster.config_kI(0, RobotMap.MMLeftkI, 10);
		leftMaster.config_kD(0, RobotMap.MMLeftkD, 10);
    }
    
    public void setPIDRight(double kF, double kP, double kI, double kD)
    {
    	RobotMap.MMRightkD = kD;
    	RobotMap.MMRightkF = kF;
    	RobotMap.MMRightkI = kI;
    	RobotMap.MMRightkP = kP;	
    	rightMaster.config_kF(0, RobotMap.MMRightkF, 10);
		rightMaster.config_kP(0, RobotMap.MMRightkP, 10);
		rightMaster.config_kI(0, RobotMap.MMRightkI, 10);
		rightMaster.config_kD(0, RobotMap.MMRightkD, 10);
    }
    
    public void motionMagic(int leftTargetTicks, int rightTargetTicks)
    {
    	leftSlave1.follow(leftMaster);
    	leftSlave2.follow(leftMaster);
    	rightSlave1.follow(rightMaster);
    	rightSlave2.follow(rightMaster);
    	leftMaster.set(ControlMode.MotionMagic, leftTargetTicks);
    	rightMaster.set(ControlMode.MotionMagic, rightTargetTicks);
    }
    
    
/*
    //returns true if finished
    public boolean followTrajectory(EncoderFollower left, EncoderFollower right)
	{
		left.configureEncoder(getEncoderL(), 128, RobotMap.wheelDiameterFeet);
		right.configureEncoder(getEncoderL(), 128, RobotMap.wheelDiameterFeet);
		// The first argument is the proportional gain. Usually this will be quite high
		// The second argument is the integral gain. This is unused for motion profiling
		// The third argument is the derivative gain. Tweak this if you are unhappy with the tracking of the trajectory
		// The fourth argument is the velocity ratio. This is 1 over the maximum velocity you provided in the
		//      trajectory configuration (it translates m/s to a -1 to 1 scale that your motors can read)
		// The fifth argument is your acceleration gain. Tweak this if you want to get to a higher or lower speed quicker
		left.configurePIDVA(0,0,0, 1 / MAX_VELOCITY, 0);
		right.configurePIDVA(0,0,0, 1 / MAX_VELOCITY, 0);
        double outputL = left.calculate(getEncoderL());
        double outputR = right.calculate(getEncoderR());
        double turn = 0;
        if (RobotMap.hasGyro)
        {
            double gyro_heading = Robot.gyro.getTrueYaw();    // Assuming the gyro is giving a value in degrees
            double desired_heading = Pathfinder.r2d(left.getHeading());  // Should also be in degrees
            double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
            turn = 0.8 * (-1.0 / 80.0) * angleDifference;
        }
        Robot.drivetrain.drive(outputL + turn, outputR - turn);
        return left.isFinished() && right.isFinished();
	}
 */
    public void motionProfile(Traj[] trajRaw)
    {
		final int maxTicksPer100msLeft = 0;
		final int maxTicksPer100msRight = 0;

		leftMaster.config_kF(0, 1023/maxTicksPer100msLeft, 0);
		leftMaster.config_kP(0, 0, 0);
		leftMaster.config_kI(0, 0, 0);
		leftMaster.config_kD(0, 0, 0);
//		leftMaster.configMotionProfileTrajectoryPeriod(10, 0);
		leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);

		rightMaster.config_kF(0, 1023/maxTicksPer100msRight, 0);
		rightMaster.config_kP(0, 0, 0);
		rightMaster.config_kI(0, 0, 0);
		rightMaster.config_kD(0, 0, 0);
//		rightMaster.configMotionProfileTrajectoryPeriod(10, 0);
		rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
		rightMaster.set(ControlMode.MotionProfile, SetValueMotionProfile.Enable.value);
    }
    
    public void stopMotionProfile() {
    	leftMaster.clearMotionProfileTrajectories();
    	rightMaster.clearMotionProfileTrajectories();
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
//    		rightSlave1.set(rightSpeed);
    		rightMaster.set(ControlMode.PercentOutput, rightSpeed);
//    		rightSlave2.set(rightSpeed);
//    		leftSlave1.set(leftSpeed);
    		leftMaster.set(ControlMode.PercentOutput, leftSpeed);
//    		leftSlave2.set(leftSpeed);
    	}
    	else drive.arcadeDrive(throttle,twist, squared);
    }
    
    public void drive(double l, double r)
    {
    	if(RobotMap.compbot)
    	{
//    		rightSlave1.set(-r);
    		rightMaster.set(ControlMode.PercentOutput, -r);
//    		rightSlave2.set(-r);
//    		leftSlave1.set(l);
    		leftMaster.set(ControlMode.PercentOutput, l);
//    		leftSlave2.set(l);
    	}
    	else if(RobotMap.flatbot) drive.tankDrive(-l, r);
    	else drive.tankDrive(l, r);
    }
    
    public double getEncoderRFeet()
    {
    	if(RobotMap.flatbot) return getEncoderR() * RobotMap.FeetPerTickFlat;
    	else return getEncoderR() * RobotMap.FeetPerTick;
    }
   // 
    public double getEncoderLFeet()
    {
    	if(RobotMap.flatbot) return getEncoderL() * RobotMap.FeetPerTickFlat;
    	else return getEncoderL() * RobotMap.FeetPerTick;
    }
    
    public int getEncoderR()
    {
    	try {
    		return encoderR.get();
    	} catch(Exception e) {}
    	return 0;
    }
    
    public int getEncoderL()
    {
    	try {
    		return encoderL.get();
    	} catch(Exception e) {}
    	return 0;
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
