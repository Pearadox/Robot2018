package org.usfirst.frc.team5414.robot.subsystems;

import java.io.File;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;
import org.usfirst.frc.team5414.robot.Traj;
import org.usfirst.frc.team5414.robot.commands.DrivewithJoystick;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

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
    
    private static final double MAX_VELOCITY = 4;
    private static final double MAX_ACCELERATION = 3.8;
    private static final double JERK = 16;
    double lastGyroError = 0;
    double angleOffset = 0;
    
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

  int lastEncoder = 0;

    public Drivetrain(){
    	if(RobotMap.flatbot) {
	    	encoderR = new Encoder(RobotMap.DIOencoderRaFlat, RobotMap.DIOencoderRbFlat, false, Encoder.EncodingType.k4X);
	    	encoderL = new Encoder(RobotMap.DIOencoderLaFlat, RobotMap.DIOencoderLbFlat, false, Encoder.EncodingType.k4X);
	    	encoderR.reset();
	    	encoderL.reset();
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
	    	encoderR.setDistancePerPulse(RobotMap.MetersPerTick);
	    	encoderL.setDistancePerPulse(RobotMap.MetersPerTick);
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
			
			
    	}
    }
    
    public EncoderFollower[] pathSetup(Waypoint[] path) {
        EncoderFollower left = new EncoderFollower();
        EncoderFollower right = new EncoderFollower();
        Trajectory.Config cfg = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH,
                .02, MAX_VELOCITY, MAX_ACCELERATION, JERK);
        String pathHash = String.valueOf(path.hashCode());
        Trajectory toFollow;
        File trajectory = new File("/home/lvuser/paths/" + pathHash + ".csv");
        if (!trajectory.exists()) 
        {
            toFollow = Pathfinder.generate(path, cfg);
            Pathfinder.writeToCSV(trajectory, toFollow);
            System.out.println(pathHash + ".csv not found, wrote to file");
        } 
        else 
        {
            System.out.println(pathHash + ".csv read from file");
            toFollow = Pathfinder.readFromCSV(trajectory);
        }

        TankModifier modifier = new TankModifier(toFollow).modify(RobotMap.wheelBaseWidth); //CHANGE THIS
        lastGyroError = 0;
        left = new EncoderFollower(modifier.getLeftTrajectory());
        right = new EncoderFollower(modifier.getRightTrajectory());
        left.configureEncoder(getEncoderL(), (int)RobotMap.EncoderTicksPerRev, RobotMap.wheelDiameterMeters);
        right.configureEncoder(getEncoderR(), (int)RobotMap.EncoderTicksPerRev, RobotMap.wheelDiameterMeters);
        left.configurePIDVA(RobotMap.MPkP, RobotMap.MPkI, RobotMap.MPkD, 1/MAX_VELOCITY, .05);
        right.configurePIDVA(RobotMap.MPkP, RobotMap.MPkI, RobotMap.MPkD, 1/MAX_VELOCITY, 05);
        return new EncoderFollower[]{
                left, // 0
                right, // 1
        };
    }
    
    public boolean pathFollow(EncoderFollower[] followers, boolean reverse) {

        EncoderFollower left = followers[0];
        EncoderFollower right = followers[1];
        double l;
        double r;
        double localGp = .02;
        if (!reverse) {
            localGp *= -1;

            l = left.calculate(-getEncoderL());
            r = right.calculate(-getEncoderR());
        } else {
            l = left.calculate(getEncoderL());
            r = right.calculate(getEncoderR());
        }

        double gyro = reverse ? -Robot.gyro.getYaw() - angleOffset : Robot.gyro.getYaw() + angleOffset;

        double desiredHeading = Pathfinder.r2d(left.getHeading());
        double angleDifference = Pathfinder.boundHalfDegrees(((desiredHeading - gyro)+36000)%360-180);
        double turn = localGp * angleDifference;
        
        if (!reverse) {
            drive(l + turn, r - turn);
        } else {
            drive(-l + turn, -r - turn);
        }

        if (left.isFinished() && right.isFinished()) {
            angleOffset = angleDifference;
        }
        SmartDashboard.putNumber("MP Left Enc Difference", left.getSegment().x-getEncoderL());
        return left.isFinished() && right.isFinished();
    }
    
    /*
     * Stitches flatline caused by squaring inputs
     */
    public void pearDrive(double throttle, double twist)
    {
    	double f = .25;
    	throttle = Math.copySign(((throttle * throttle + Math.abs(2*throttle*f)) / (1+2*f)), throttle);
    	twist = Math.copySign(((twist * twist + Math.abs(2*twist *f)) / (1+2*f)), twist);
    	arcadeDrive(throttle, twist*.7, false);
    	if(Robot.oi.getJoystick().getRawButtonPressed(11)) zeroEncoders();
    	if(Robot.oi.getJoystick().getRawButton(11)) 
    	{
    		System.out.print(getEncoderL() + " " + getEncoderR() + " ");
    	}
    	if(Robot.oi.getJoystick().getRawButtonReleased(11)) System.out.println("----------------------------");
    }
    
    public void cheesyDrive(double throttle, double wheel,
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
	  overPower = 0.0;
	  angularPower = Math.abs(throttle) * wheel * sensitivity - mQuickStopAccumlator;
	  if (mQuickStopAccumlator > 1) {
	      mQuickStopAccumlator -= 1;
	  } else if (mQuickStopAccumlator < -1) {
	      mQuickStopAccumlator += 1;
	  } else {
	      mQuickStopAccumlator = 0.0;
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
    	if(Math.abs(stick.getRawAxis(1)) < .18)		//Setting deadzone for the y-axis
    	{
    		ax2 = 0;
    	}
    	else
    	{
    		ax2 = stick.getRawAxis(1);
    	}
    	
    	if(Robot.oi.getJoystick().getRawButtonPressed(11)) zeroEncoders();
    	if(Robot.oi.getJoystick().getRawButton(11)) 
    	{
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
    	if (squared) {
			throttle = Math.copySign(throttle * throttle * throttle, throttle);
		    twist = -Math.copySign(twist * twist * twist, twist);
		}
    	if(RobotMap.compbot)
    	{
    		double leftSpeed = throttle - twist;
    		double rightSpeed = -throttle - twist;
    		rightMaster.set(ControlMode.PercentOutput, rightSpeed);
    		leftMaster.set(ControlMode.PercentOutput, leftSpeed);
    	}
    	else drive.arcadeDrive(throttle,twist, squared);
    }
    
    public void drive(double l, double r)
    {
    	if(RobotMap.compbot)
    	{
    		rightMaster.set(ControlMode.PercentOutput, -r);
    		leftMaster.set(ControlMode.PercentOutput, l);
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
    		int get = encoderR.get();
//    		get /= 2;  //only for practice bot (it has 256 tick encoders compared to 128 ticks on the comp bot)
    		return get;
    	} catch(Exception e) {}
    	return 0;
    }
    
    public int getEncoderL()
    {
    	try {
    		int get = encoderL.get();
//    		get /= 2;  //only for practice bot (it has 256 tick encoders compared to 128 ticks on the comp bot)
    		return get;
    	} catch(Exception e) {}
    	return 0;
    }
    
    public double getSpeedR() // m/s 
    {
    	try {
    		return encoderR.getRate();
    	} catch(Exception e) {}
    	return 0;
    }
    
    public double getSpeedL() // m/s 
    {
    	try {
    		return encoderL.getRate();
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
	
	public void setBrake() {
		rightMaster.setNeutralMode(NeutralMode.Brake);
		rightSlave1.setNeutralMode(NeutralMode.Brake);
		rightSlave2.setNeutralMode(NeutralMode.Brake);
		leftMaster.setNeutralMode(NeutralMode.Brake);
		leftSlave1.setNeutralMode(NeutralMode.Brake);
		leftSlave2.setNeutralMode(NeutralMode.Brake);
	}
	
	public void setCoast() {
		rightMaster.setNeutralMode(NeutralMode.Brake);
		rightSlave1.setNeutralMode(NeutralMode.Coast);
		rightSlave2.setNeutralMode(NeutralMode.Coast);
		leftMaster.setNeutralMode(NeutralMode.Brake);
		leftSlave1.setNeutralMode(NeutralMode.Coast);
		leftSlave2.setNeutralMode(NeutralMode.Coast);
	}
    
    public void initDefaultCommand() {
    	setDefaultCommand(new DrivewithJoystick());
    }
}
