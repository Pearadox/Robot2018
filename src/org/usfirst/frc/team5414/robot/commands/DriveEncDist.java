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
    	this((int)(d / RobotMap.LengthPerTickMetersFlat), (int)(d / RobotMap.LengthPerTickMetersFlat));
//    	distance *= .92;
//    	setTimeout(4);
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
			RobotMap.plybotRkP = Robot.prefs.getDouble("PlyEnc R kP", RobotMap.plybotRkP);
			RobotMap.plybotRkI = Robot.prefs.getDouble("PlyEnc R kI", RobotMap.plybotRkI);
			RobotMap.plybotRkD = Robot.prefs.getDouble("PlyEnc R kD", RobotMap.plybotRkD);
		}
    	initialTime = Timer.getFPGATimestamp();
    	lastTime = Timer.getFPGATimestamp();
    	Robot.drivetrain.zeroEncoders();
    	initialEncoderTicks = Robot.drivetrain.getEncoderL();
    	if(RobotMap.flatbot)
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
    	if(RobotMap.flatbot) changeInAngle = Robot.gyro.getYaw()-originalAngle;
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
    		double leftI = errorSumLeft * (RobotMap.flatbot ? RobotMap.flatbotkP : RobotMap.plybotLkI);
    		double rightI = errorSumRight * (RobotMap.flatbot ? RobotMap.flatbotkP : RobotMap.plybotRkI);
    		double leftD = ((leftEnc.get(recordedLoops) - lastDesiredLeft) - (leftError - lastLeftError)) * (RobotMap.flatbot ? RobotMap.flatbotkP : RobotMap.plybotLkD); //dSetpoint - dError , prevents derivative kick
    		double rightD = ((rightEnc.get(recordedLoops) - lastDesiredRight) - (rightError - lastRightError)) * (RobotMap.flatbot ? RobotMap.flatbotkP : RobotMap.plybotRkD);//dSetpoint - dError
    		double leftOutput = leftP + leftI - leftD; 
    		double rightOutput = rightP + rightI - rightD;
    		
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
    		Traj trajectory = getTrajectory(Math.abs(targetFeet), elapsedTime);
    		
//	    	if(Robot.drivetrain.getEncoderL() < (initDistance + distance) && targetFeet >= 0) //positive
//	    	{
//	    		double error = trajectory.distance - (Robot.drivetrain.getEncoderL() - initialEncoderTicks) * RobotMap.LengthPerTickMetersFlat;
//	    		double errorDeriv = (error - lastError) / lastTime;
//	    		double motorSpeed = Kv * trajectory.velocity + Ka * trajectory.acceleration + Kp * error + Kd * errorDeriv;
//	    		Robot.drivetrain.arcadeDrive(changeInAngle * drivep, motorSpeed, false);
//	    		lastError = error;
//	    		lastTime = Timer.getFPGATimestamp();
//	    	}
//	    	else if(Robot.drivetrain.getEncoderL() > (initDistance + distance) && targetFeet >= 0)
//	    	{
//	    		Robot.drivetrain.stop();
//	    	}
//	    	
//	    	else if(Robot.drivetrain.getEncoderL() > (distance + initDistance) && targetFeet < 0) //negative
//	    	{
//	    		double error = (Robot.drivetrain.getEncoderL() - initialEncoderTicks) * RobotMap.LengthPerTickMetersFlat - trajectory.distance;
//	    		double errorDeriv = (error - lastError) / lastTime;
//	    		double motorSpeed = Kv * trajectory.velocity + Ka * trajectory.acceleration + Kp * error + Kd * errorDeriv;
//	    		Robot.drivetrain.arcadeDrive(changeInAngle * drivep, -motorSpeed, false);
//	    		lastError = error;
//	    		lastTime = Timer.getFPGATimestamp();
//	    	}
//	    	else if(Robot.drivetrain.getEncoderL() < (distance + initDistance) && targetFeet < 0)
//	    	{
//	    		Robot.drivetrain.stop();
//	    	}
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
