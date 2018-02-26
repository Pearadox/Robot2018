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
	ArrayList<Double> leftEnc = null;
	ArrayList<Double> rightEnc = null;
	double errorSumLeft = 0;
	double errorSumRight = 0;
	int recordedLoops = 0;
	double lastLeftError = 0;
	double lastRightError = 0; 
	boolean tracked = false;
	double changeInTime = 0;
	double timeStep = .05;
	int loops = 0;
	int settleLoops = 0;
	
	final double Kv = 1 / CRUISE_VELOCITY;
	final double Ka = .06;
	final double Kp = 3.5;
	final double Kd = 200;

    public DriveEncDist(double d) { //meters
    	targetFeet = d;
	}
    
    public DriveEncDist(double left, double right)
    {
    	requires(Robot.drivetrain);
    	leftEnc = new ArrayList<Double>();
    	rightEnc = new ArrayList<Double>();
    	leftEnc.add(left);
    	rightEnc.add(right);
    	tracked = true;
    }
    
    public DriveEncDist(ArrayList<Double> left, ArrayList<Double> right)
    {
    	leftEnc = left;
    	rightEnc = right;
    	tracked = true;
    }

	protected void initialize() {
		if(RobotMap.flatbot)
		{
			RobotMap.flatbotkP = Robot.prefs.getDouble("FlatEnc kP", RobotMap.flatbotkP);
			RobotMap.flatbotkI = Robot.prefs.getDouble("FlatEnc kI", RobotMap.flatbotkI);
			RobotMap.flatbotkD = Robot.prefs.getDouble("FlatEnc kD", RobotMap.flatbotkD);
		}
		else if(!RobotMap.flatbot)
		{
			RobotMap.plybotLkP = Robot.prefs.getDouble("PlyEnc L kP", RobotMap.plybotLkP);
			RobotMap.plybotLkI = Robot.prefs.getDouble("PlyEnc L kI", RobotMap.plybotLkI);
			RobotMap.plybotLkD = Robot.prefs.getDouble("PlyEnc L kD", RobotMap.plybotLkD);
			RobotMap.plybotLkF = Robot.prefs.getDouble("PlyEnc L kF", RobotMap.plybotLkF);			
			RobotMap.plybotRkP = Robot.prefs.getDouble("PlyEnc R kP", RobotMap.plybotRkP);
			RobotMap.plybotRkI = Robot.prefs.getDouble("PlyEnc R kI", RobotMap.plybotRkI);
			RobotMap.plybotRkD = Robot.prefs.getDouble("PlyEnc R kD", RobotMap.plybotRkD);
			RobotMap.plybotRkF = Robot.prefs.getDouble("PlyEnc R kF", RobotMap.plybotRkF);
		}
    	initialTime = Timer.getFPGATimestamp();
    	lastTime = Timer.getFPGATimestamp();
    	Robot.drivetrain.zeroEncoders();
    	initialEncoderTicks = Robot.drivetrain.getEncoderL();
    	if(RobotMap.hasGyro)
    	{
    		Robot.gyro.zeroYaw();
    		originalAngle = Robot.gyro.getYaw();
    	}
    	initDistance = Robot.drivetrain.getEncoderLFeet();
    	settleLoops = 1;
    }

    protected void execute() {
    	tracked= true;
    	double changeInAngle = 0;
    	if(RobotMap.hasGyro) changeInAngle = Robot.gyro.getYaw()-originalAngle;
    	double elapsedTime = Timer.getFPGATimestamp() - initialTime;
    	changeInTime = elapsedTime;
    	SmartDashboard.putBoolean("Tracked", tracked);
    	if(tracked)
    	{
    		double currentLeft = Robot.drivetrain.getEncoderL();
    		double currentRight = Robot.drivetrain.getEncoderR();
    		double leftError = leftEnc.get(recordedLoops) - currentLeft;
    		double rightError = rightEnc.get(recordedLoops) - currentRight;
    		errorSumLeft += leftError;
    		errorSumRight += rightError;
    		double lastDesiredLeft = recordedLoops > 0 ? leftEnc.get(recordedLoops-1) : 0;
    		double lastDesiredRight = recordedLoops > 0 ? rightEnc.get(recordedLoops-1) : 0;
    		
//    		if(Math.abs(leftError) <= 3) errorSumLeft = 0;
//    		if(Math.abs(rightError) <= 3) errorSumRight = 0;
    		
    		//PID Calculations
    		double leftP = leftError * (RobotMap.flatbot ? RobotMap.flatbotkP : RobotMap.plybotLkP);
    		double rightP = rightError * (RobotMap.flatbot ? RobotMap.flatbotkP : RobotMap.plybotRkP);
    		double leftI = errorSumLeft * (RobotMap.flatbot ? RobotMap.flatbotkI : RobotMap.plybotLkI);
    		double rightI = errorSumRight * (RobotMap.flatbot ? RobotMap.flatbotkI : RobotMap.plybotRkI);
    		double leftD = ((leftEnc.get(recordedLoops) - lastDesiredLeft) - (leftError - lastLeftError)) * (RobotMap.flatbot ? RobotMap.flatbotkD : RobotMap.plybotLkD); //dSetpoint - dError , prevents derivative kick
    		double rightD = ((rightEnc.get(recordedLoops) - lastDesiredRight) - (rightError - lastRightError)) * (RobotMap.flatbot ? RobotMap.flatbotkD : RobotMap.plybotRkD);//dSetpoint - dError
    		double leftOutput = leftP + leftI - leftD + RobotMap.plybotLkF; 
    		double rightOutput = rightP + rightI - rightD + RobotMap.plybotRkF;
    		
    		Robot.drivetrain.drive(leftOutput, rightOutput);
    		
    		SmartDashboard.putNumber("current L", currentLeft);
    		SmartDashboard.putNumber("current R", currentRight);
    		SmartDashboard.putNumber("desired L", leftEnc.get(recordedLoops));
    		SmartDashboard.putNumber("desired R", rightEnc.get(recordedLoops));
    		SmartDashboard.putNumber("output L", leftOutput);
    		SmartDashboard.putNumber("output R", rightOutput);
    		SmartDashboard.putNumber("error L", leftError);
    		SmartDashboard.putNumber("error R", rightError);
    		
    		lastLeftError = leftError;
    		lastRightError = rightError;
    	}
    	else
    	{
    		Traj[] trajectory = getTrajectory(targetFeet, .01);
    	}
    }

    protected boolean isFinished() {
    	if(settleLoops == 0)
    	{
    		recordedLoops++;
    		settleLoops = 2;
    		if(recordedLoops >= leftEnc.size()) return true;
    		return false;
    	}
    	if(tracked)
    	{
    		if(leftEnc.size() == 1)
    		{
    			if(Math.abs(lastLeftError) < 20 && Math.abs(lastRightError) < 20)
    			{
    				return true;
    			}
    			else return false;
    		}
    		else if(Math.abs(lastLeftError) < 130 && Math.abs(lastRightError) < 130)
    		{
    			recordedLoops++;
    			if(recordedLoops >= leftEnc.size()) return true;
    		}
    		return false;
    	}
    	return false;
    }

    protected void end() {
    	tracked = false;
    	recordedLoops = 0;
    	lastError = 0;
    	changeInTime = 0;
    	errorSumLeft = 0;
    	errorSumRight = 0;
    	lastLeftError = 0;
    	lastRightError = 0;
    }

    protected void interrupted() {
    	end();
    }
    
    public static Traj[] getTrajectory(double distance, double interval) //meters, seconds
    {
        double accelTime = CRUISE_VELOCITY / ACCELERATION;
        double accelDistance = .5 * ACCELERATION * accelTime * accelTime * Math.copySign(1, distance);
        ArrayList<Traj> list = new ArrayList<>();
        
        if(Math.abs(accelDistance * 2) <= Math.abs(distance)) //trapezoidal
        {
            double rectangleDistance = (distance - accelDistance*2) * Math.copySign(1, distance);
            double cruiseTime = Math.abs(rectangleDistance / CRUISE_VELOCITY);

           
            for(double time = 0 ;; time += interval)
            {
                if(time < accelTime)
                {
                    double s = ACCELERATION * time * Math.copySign(1, distance);
                    double a = ACCELERATION * Math.copySign(1, distance);
                    double d = .5 * s * time;
                    list.add(new Traj(s, d, a));
                }
                else if(time <= cruiseTime + accelTime)
                {
                	double currentCruiseTime = time - accelTime;
                	double s = CRUISE_VELOCITY;
                	double a = 0;
                	double d = accelDistance + currentCruiseTime * CRUISE_VELOCITY;
                	list.add(new Traj(s, d, a));
                }
                else if(time <= cruiseTime + 2*accelTime)
                {
                    double timeAfterDecelerationStarted = time - cruiseTime - accelTime;
                    double decelVelocity = (CRUISE_VELOCITY - ACCELERATION * timeAfterDecelerationStarted) * Math.copySign(1, distance);
                    if(decelVelocity < 0) decelVelocity = 0;
                    double s = decelVelocity;
                    double a = -ACCELERATION * Math.copySign(1, distance);
                    if(s == 0) a = 0;
                    double d = distance - (accelTime - timeAfterDecelerationStarted)*s/2;
                    list.add(new Traj(s, d, a));
                }
                else break;
            }
            
        }
        else //triangular
        {
            accelDistance = distance / 2. * Math.copySign(1, distance);
            accelTime = Math.sqrt(2 * accelDistance  * Math.copySign(1, distance)/ ACCELERATION);
            for(double time = 0 ;; time+=interval)
            {
	            if(time <= accelTime)
	            {
	                double s = ACCELERATION * time * Math.copySign(1, distance);
	                double a = ACCELERATION * Math.copySign(1, distance);
	                double d = .5 * s * time;
	                list.add(new Traj(s, d, a));
	            }
	            else if(time <= 2 * accelTime)
	            {
	                double peakVelocity = ACCELERATION * accelTime * Math.copySign(1, distance);
	                double decelVelocity = peakVelocity - ACCELERATION * (time - accelTime) * Math.copySign(1, distance);
	                if(decelVelocity < 0) decelVelocity = 0;
	                double s = decelVelocity;
	                double a = -ACCELERATION * Math.copySign(1, distance);
	                if(decelVelocity == 0) a = 0;
	                double d = distance - (accelTime * 2 - time) * s / 2;
	                list.add(new Traj(s, d, a));
	            }
	            else break;
        	}
        }
        return list.toArray(new Traj[list.size()]);
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