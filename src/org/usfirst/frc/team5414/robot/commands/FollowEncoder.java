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
	int settleLoops = 0;
    
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
    	leftEnc = left;
    	rightEnc = right;
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
    	settleLoops = 1;
    }

    protected void execute() {
		double currentLeft = Robot.drivetrain.getEncoderL();
		double currentRight = Robot.drivetrain.getEncoderR();
		double leftError = leftEnc.get(recordedLoops) - currentLeft;
		double rightError = rightEnc.get(recordedLoops) - currentRight;
		errorSumLeft += leftError;
		errorSumRight += rightError;
		double lastDesiredLeft = recordedLoops > 0 ? leftEnc.get(recordedLoops-1) : 0;
		double lastDesiredRight = recordedLoops > 0 ? rightEnc.get(recordedLoops-1) : 0;
		
		//PID Calculations
		double leftP = leftError * (RobotMap.flatbot ? RobotMap.flatbotkP : RobotMap.plybotLkP);
		double rightP = rightError * (RobotMap.flatbot ? RobotMap.flatbotkP : RobotMap.plybotRkP);
		double leftI = errorSumLeft * (RobotMap.flatbot ? RobotMap.flatbotkI : RobotMap.plybotLkI);
		double rightI = errorSumRight * (RobotMap.flatbot ? RobotMap.flatbotkI : RobotMap.plybotRkI);
		double leftD = ((leftEnc.get(recordedLoops) - lastDesiredLeft) - (leftError - lastLeftError)) * 
				(RobotMap.flatbot ? RobotMap.flatbotkD : RobotMap.plybotLkD); //dSetpoint - dError , prevents derivative kick
		double rightD = ((rightEnc.get(recordedLoops) - lastDesiredRight) - (rightError - lastRightError)) * 
				(RobotMap.flatbot ? RobotMap.flatbotkD : RobotMap.plybotRkD);//dSetpoint - dError
		double leftOutput = leftP + leftI - leftD; 
		double rightOutput = rightP + rightI - rightD;
		leftOutput += Math.copySign(RobotMap.plybotLkF, leftError);
		rightOutput += Math.copySign(RobotMap.plybotRkF, rightError);
		
		Robot.drivetrain.drive(leftOutput, rightOutput);
		
		SmartDashboard.putNumber("output L", leftOutput);
		SmartDashboard.putNumber("output R", rightOutput);
		SmartDashboard.putNumber("error L", leftError);
		SmartDashboard.putNumber("error R", rightError);
		
		lastLeftError = leftError;
		lastRightError = rightError;
    }

    protected boolean isFinished() {
    	if(settleLoops == 0)
        {
        	recordedLoops++;
        	settleLoops = 5;
        	if(recordedLoops >= leftEnc.size()) return true;
        	return false;
        }
    	if(leftEnc.size() == 1 || settleLoops == leftEnc.size()-1)
    	{
    		if(Math.abs(lastLeftError) < 3 && Math.abs(lastRightError) < 3)
    		{
				return true;
			}
			else return false;
		}
		else if(Math.abs(lastLeftError) < 70 && Math.abs(lastRightError) < 70)
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
    }

    protected void interrupted() {
    	end();
    }
}
