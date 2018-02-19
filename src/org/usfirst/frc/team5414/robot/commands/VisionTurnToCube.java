package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class VisionTurnToCube extends Command {

	final int timeoutLoops = 10; //loops * 20ms = milliseconds of timeout
	final int settleLoopsGoal = 2;
	int settleLoops = 0;
	int currentTimeoutLoops = 0;
	double targetArea = 12;
	double errorSum;
	double lastError;
	double lastAreaError;
	
    public VisionTurnToCube() {
    	requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	errorSum = 0;
    	lastError = 0;
    	lastAreaError = 0;
    	settleLoops = 0;
    	RobotMap.turnLimekP = Robot.prefs.getDouble("Limelight kP", RobotMap.turnLimekP);
    	RobotMap.turnLimekD = Robot.prefs.getDouble("Limelight kD", RobotMap.turnLimekD);
    	RobotMap.forwardLimekP = Robot.prefs.getDouble("Limelight Forward kP", RobotMap.forwardLimekP);
    	RobotMap.forwardLimekD = Robot.prefs.getDouble("Limelight Forward kD", RobotMap.forwardLimekD);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//turn to cube
    	if(!Robot.limelight.hasTarget()) //if robot doesn't see a cube, constantly turn left 
    	{
    		Robot.drivetrain.drive(.55, -.55);
    		return;
    	}
    	currentTimeoutLoops = 0;
    	double error = Robot.limelight.getX();
    	errorSum += error;
    	if(Math.abs(error) <= 2) errorSum = 0;
    	double F = error > 0 ? 0.31 : -0.31;
    	double P = error * RobotMap.turnLimekP;
    	double I = errorSum * RobotMap.turnLimekI;
    	double D = (lastError - error) * RobotMap.turnLimekD;
    	double output = P + I - D + F;
    	double leftOutput = -output;
    	double rightOutput = output;
    	
    	double areaError = targetArea - Robot.limelight.getArea();
    	double forwardP = areaError * RobotMap.forwardLimekP;
    	double forwardD = (lastError - areaError) * RobotMap.forwardLimekD;
    	double forwardOutput = forwardP - forwardD;
    	leftOutput += forwardOutput;
    	rightOutput += forwardOutput;
    	
    	Robot.drivetrain.drive(leftOutput, rightOutput);
    	lastError = error;
    	lastAreaError = areaError;
    	SmartDashboard.putNumber("Vision Error", error);
    	SmartDashboard.putNumber("Vision OutputL", leftOutput);
    	SmartDashboard.putNumber("Vision OutputR", rightOutput);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
//    	if(!Robot.limelight.hasTarget()) return ++currentTimeoutLoops == 10;
    	/*
    	if(lastError == 0) return false;
        if(lastAreaError < -1)
        {
        	if(settleLoops++ == settleLoopsGoal) return true;
        	else return false;
        }
        else return false;
        */
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivetrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
