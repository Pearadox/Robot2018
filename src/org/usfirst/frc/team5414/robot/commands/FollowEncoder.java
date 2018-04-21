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

public class FollowEncoder extends Command{
	
	ArrayList<Double> leftEnc = null;
	ArrayList<Double> rightEnc = null;
	double errorSumLeft = 0;
	double errorSumRight = 0;
	int recordedLoops = 0;
	double lastLeftError = 0;
	double lastRightError = 0;
	int lastLeft;
	int lastRight;
	double timeout;
    
    public FollowEncoder(double left, double right)
    {
    	requires(Robot.drivetrain);
    	leftEnc = new ArrayList<Double>();
    	rightEnc = new ArrayList<Double>();
    	leftEnc.add(left);
    	rightEnc.add(right);
    }
    
    public FollowEncoder(ArrayList<Double> left, ArrayList<Double> right)
    {
    	this(left, right, 999);
    }
    
    public FollowEncoder(ArrayList<Double> left, ArrayList<Double> right, double timeout)
    {
    	leftEnc = left;
    	rightEnc = right;
    	this.timeout = timeout;
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
    	Robot.drivetrain.zeroEncoders();
    	Robot.drivetrain.setCoast();
    	lastLeft = 0;
    	lastRight = 0;
    	setTimeout(timeout);
    }

    protected void execute() {
		int currentLeft = Robot.drivetrain.getEncoderL();
		int currentRight = Robot.drivetrain.getEncoderR();
		double leftError = leftEnc.get(recordedLoops) - currentLeft;
		double rightError = rightEnc.get(recordedLoops) - currentRight;
		errorSumLeft += leftError;
		errorSumRight += rightError;
		
		//PID Calculations
		double leftP = leftError * (RobotMap.flatbot ? RobotMap.flatbotkP : RobotMap.plybotLkP);
		double rightP = rightError * (RobotMap.flatbot ? RobotMap.flatbotkP : RobotMap.plybotRkP);
		double leftI = errorSumLeft * (RobotMap.flatbot ? RobotMap.flatbotkI : RobotMap.plybotLkI);
		double rightI = errorSumRight * (RobotMap.flatbot ? RobotMap.flatbotkI : RobotMap.plybotRkI);
		double leftD = (currentLeft - lastLeft) * (RobotMap.flatbot ? RobotMap.flatbotkD : RobotMap.plybotLkD); //dInput
		double rightD = (currentRight - lastRight)  * (RobotMap.flatbot ? RobotMap.flatbotkD : RobotMap.plybotRkD);//dInput
		double leftOutput = leftP + leftI - leftD; 
		double rightOutput = rightP + rightI - rightD;
		leftOutput += Math.copySign(RobotMap.plybotLkF, leftError);
		rightOutput += Math.copySign(RobotMap.plybotRkF, rightError);
		
		Robot.drivetrain.drive(leftOutput, rightOutput);
		
		SmartDashboard.putNumber("output L", leftOutput);
		SmartDashboard.putNumber("output R", rightOutput);
		SmartDashboard.putNumber("error L", leftError);
		SmartDashboard.putNumber("error R", rightError);

		lastLeft = currentLeft;
		lastRight = currentRight;
		lastLeftError = leftError;
		lastRightError = rightError;
    }

    protected boolean isFinished() {
    	if(isTimedOut()) return true;
    	if(leftEnc.size() == 1 || recordedLoops == leftEnc.size()-1)
    	{
    		if(Math.abs(lastLeftError) < 20 && Math.abs(lastRightError) < 20)
    		{
				return true;
			}
			else return false;
		}
		else if(Math.abs(lastLeftError) < 10 && Math.abs(lastRightError) < 10)
		{
			recordedLoops++;
			if(recordedLoops >= leftEnc.size()) return true;
		}
		return false;
    }

    protected void end() {
    	Robot.drivetrain.stop();
    	recordedLoops = 0;
    	errorSumLeft = 0;
    	errorSumRight = 0;
    	lastLeftError = 0;
    	lastRightError = 0;
    	Robot.turnIsDone = true;
    	Robot.drivetrain.setBrake();
    }

    protected void interrupted() {
    	end();
    }
}