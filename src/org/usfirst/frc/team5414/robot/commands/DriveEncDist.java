package org.usfirst.frc.team5414.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Trajectory;
//import jaci.pathfinder.Trajectory;
//import jaci.pathfinder.modifiers.TankModifier;
import jaci.pathfinder.modifiers.TankModifier;

public class DriveEncDist extends Command{
	double initDistance;
	double distance;
	double originalAngle;
	double targetFeet;
	double drivep = .15;
	double Inputspeed;
	static final double CRUISE_VELOCITY = 14;
	static final double ACCELERATION = 3;
	double initialTime = 0;
	double initialEncoderTicks = 0;
	double lastError = 0;
	double lastTime = 0;
	boolean presetTrajectory = false;
	Trajectory trajLeft = null;
	Trajectory trajRight = null;
	Traj[] leftArr = null;
	Traj[] rightArr = null;
	ArrayList<MotorPair> follow = null;
	int recordedLoops = 0;
	boolean recorded = false;
	double changeInTime = 0;
	double timeStep = .05;
	
	final double Kv = 1 / CRUISE_VELOCITY;
	final double Ka = .06;
	final double Kp = 3.5;
	final double Kd = 200;

    public DriveEncDist(double d) { //meters
    	targetFeet = d;
    	distance = d / RobotMap.LengthPerTickMeters;
    	distance *= .92;
    	setTimeout(4);
    	requires(Robot.drivetrain);
	}
    
    public DriveEncDist(double d, double s) {}
    public DriveEncDist(double d, double s, double t) {}
    
    public DriveEncDist(ArrayList<MotorPair> record)
    {
    	follow = record;
    	recorded = true;
    }
    
//    public DriveEncDist(Trajectory preset)
//    {
//    	TankModifier modifier = new TankModifier(preset);
//    	modifier.modify(RobotMap.WheelBaseWidth);
//    	trajLeft = modifier.getLeftTrajectory();
//    	trajRight = modifier.getRightTrajectory();
//    	presetTrajectory = true;
//    }

    public DriveEncDist(Traj[] left, Traj[] right) {
		leftArr = left;
		rightArr = right;
		presetTrajectory = true;
	}

	protected void initialize() {
    	
    	initialTime = Timer.getFPGATimestamp();
    	lastTime = Timer.getFPGATimestamp();
//    	initialEncoderTicks = Robot.drivetrain.getEncoderBL();
//    	Robot.gyro.zeroYaw();
//    	originalAngle = Robot.gyro.getYaw();
//    	initDistance = Robot.drivetrain.getEncoderBL();
    }

    protected void execute() {
//    	double changeInAngle = Robot.gyro.getYaw()-originalAngle;
    	double changeInAngle = 0;
    	double elapsedTime = Timer.getFPGATimestamp() - initialTime;
    	changeInTime = elapsedTime;
    	if(recorded)
    	{
    		try
    		{
    			MotorPair mp = follow.get(recordedLoops);
    			System.out.println(mp);
    			Robot.drivetrain.arcadeDrive(mp.two, mp.one);
    			recordedLoops++;
    		} catch(Exception e){}
    	}
    	if(presetTrajectory)
    	{
    		Traj left = leftArr[(int)Math.round(elapsedTime / timeStep)];
    		Traj right = rightArr[(int)Math.round(elapsedTime / timeStep)];
//    		Trajectory.Segment left = trajLeft.get((int)Math.round(elapsedTime / timeStep));
//    		Trajectory.Segment right = trajRight.get((int)Math.round(elapsedTime / timeStep));
//    		double error = trajectory.distance - (Robot.drivetrain.getEncoderBL() - initialEncoderTicks) * RobotMap.LengthPerTick;
    		double error = 0;
//    		double errorDeriv = (error - lastError) / lastTime;
    		double errorDeriv = 0;
    		try {
    		double motorSpeedLeft = Kv * left.velocity + Ka * left.acceleration + Kp * error + Kd * errorDeriv;
    		double motorSpeedRight = Kv * right.velocity + Ka * right.acceleration + Kp * error + Kd * errorDeriv;
    		Robot.drivetrain.drive(motorSpeedLeft*3, motorSpeedRight*3);
    		} catch (Exception e) {}
    		lastError = error;
    		lastTime = Timer.getFPGATimestamp();
//    		DriverStation.reportWarning(left.toString(), true);
    	}
    	else
    	{
    		Traj trajectory;
    		trajectory = getTrajectory(Math.abs(targetFeet), elapsedTime);
	    	if(Robot.drivetrain.getEncoderBL() < (initDistance + distance) && targetFeet >= 0) //positive
	    	{
	    		double error = trajectory.distance - (Robot.drivetrain.getEncoderBL() - initialEncoderTicks) * RobotMap.LengthPerTickMeters;
	    		double errorDeriv = (error - lastError) / lastTime;
	    		double motorSpeed = Kv * trajectory.velocity + Ka * trajectory.acceleration + Kp * error + Kd * errorDeriv;
	    		Robot.drivetrain.arcadeDrive(changeInAngle * drivep, motorSpeed);
	    		lastError = error;
	    		lastTime = Timer.getFPGATimestamp();
	    	}
	    	else if(Robot.drivetrain.getEncoderBL() > (initDistance + distance) && targetFeet >= 0)
	    	{
	    		Robot.drivetrain.stop();
	    	}
	    	
	    	else if(Robot.drivetrain.getEncoderBL() > (distance + initDistance) && targetFeet < 0) //negative
	    	{
	    		double error = (Robot.drivetrain.getEncoderBL() - initialEncoderTicks) * RobotMap.LengthPerTickMeters - trajectory.distance;
	    		double errorDeriv = (error - lastError) / lastTime;
	    		double motorSpeed = Kv * trajectory.velocity + Ka * trajectory.acceleration + Kp * error + Kd * errorDeriv;
	    		Robot.drivetrain.arcadeDrive(changeInAngle * drivep, -motorSpeed);
	    		lastError = error;
	    		lastTime = Timer.getFPGATimestamp();
	    	}
	    	else if(Robot.drivetrain.getEncoderBL() < (distance + initDistance) && targetFeet < 0)
	    	{
	    		Robot.drivetrain.stop();
	    	}
	
//	    	SmartDashboard.putNumber("Difference In Angle", Robot.gyro.getYaw() - originalAngle);
//	    	SmartDashboard.putNumber("Desired EncDist", initDistance + distance);
//	    	SmartDashboard.putNumber("Current EncDist", Robot.drivetrain.getEncoderBL());
    	}
    }

    protected boolean isFinished() {
    	return false;
//    	if(isTimedOut()) return true;
//    	if(presetTrajectory)
//    	{
//    		if(leftArr.length * timeStep >= changeInTime)
//    			return true;
//    		else return false;
//    	}
//    	if(Robot.drivetrain.getEncoderBL() >= initDistance + distance && targetFeet > 0){
//    		return true;
//    	}
//    	else if((Robot.drivetrain.getEncoderBL() <= (initDistance + distance)) && targetFeet < 0)
//    	{
//    		return true;
//    	}
//    	else if(targetFeet == 0) return true;
//    	
//    	return false;
    }

    protected void end() {
    	Robot.drivetrain.stop();
    	presetTrajectory = false;
    	recorded = false;
    	trajLeft = null;
    	trajRight = null;
    	changeInTime = 0;
    	recordedLoops = 0;
    }

    protected void interrupted() {
    	end();
    }
    
    public static Traj getTrajectory(double distance, double time)
    {
        double accelTime = CRUISE_VELOCITY / ACCELERATION;
        double accelDistance = .5 * ACCELERATION * accelTime * accelTime;

        if(accelDistance * 2 <= distance) //trapezoidal
        {
            double rectangleDistance = distance - accelDistance*2;
            double cruiseTime = rectangleDistance / CRUISE_VELOCITY;

            if(time < accelTime)
            {
                double s = ACCELERATION * time;
                double a = ACCELERATION;
                double d = .5 * s * time;
                return new Traj(s, d, a);
            }

            if(cruiseTime + accelTime <= time)
            {
                double timeAfterDecelerationStarted = time - cruiseTime - accelTime;
                double toReturn = CRUISE_VELOCITY - ACCELERATION * timeAfterDecelerationStarted;
                if(toReturn < 0) toReturn = 0;
                double s = toReturn;
                double a = -ACCELERATION;
                if(toReturn == 0) a = 0;
                double d = distance - (accelTime - timeAfterDecelerationStarted )*s/2;
                return new Traj(s, d, a);
            }
            double s = CRUISE_VELOCITY;
            double a = 0;
            double d = accelDistance + (time - accelTime)*CRUISE_VELOCITY;
            return new Traj(s, d, a);
        }


        else //triangular
        {
            accelDistance = distance / 2.;
            accelTime = Math.sqrt(2 * accelDistance / ACCELERATION);
            if(time <= accelTime)
            {
                double s = ACCELERATION * time;
                double a = ACCELERATION;
                double d = .5 * s * time;
                return new Traj(s, d, a);
            }
            else
            {
                double peakVelocity = ACCELERATION * accelTime;
                double toReturn = peakVelocity - ACCELERATION * (time - accelTime);
                if(toReturn < 0) toReturn = 0;
                double s = toReturn;
                double a = -ACCELERATION;
                if(toReturn == 0) a = 0;
                double d = distance - (accelTime * 2 - time) * s / 2;
                return new Traj(s, d, a);
            }
        }
    }
    
}

class Traj
{

    public double velocity = 0;
    public double distance = 0;
    public double acceleration = 0;

    public Traj(double spd, double dist, double accel)
    {
        velocity = spd;
        distance = dist;
        acceleration = accel;
    }

    @Override
    public String toString()
    {
        return String.format("%.2f", velocity) + "Ft/s | " + String.format("%.2f", distance)+ " Ft | " + acceleration + " Ft/s^2";
    }
    
}

class MotorPair
{
	
	public double one = 0;
	public double two = 0;
	
	public MotorPair(double a, double b)
	{
		one = a;
		two = b;
	}
	
	public String toString()
	{
		return one + " " + two;
	}
}
