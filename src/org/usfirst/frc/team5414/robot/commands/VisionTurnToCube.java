package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *	Quickly turns to the first seen cube
 *	CAMERA IS INVERSED
 */
public class VisionTurnToCube extends Command {

	final int timeoutLoops = 10; //loops * 20ms = milliseconds of timeout
	final int settleLoopsGoal = 2;
	int settleLoops = 0;
	int currentTimeoutLoops = 0;
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
    	RobotMap.turnLimekI = Robot.prefs.getDouble("Limelight kI", RobotMap.turnLimekI);
    	RobotMap.turnLimekD = Robot.prefs.getDouble("Limelight kD", RobotMap.turnLimekD);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	try {
	    	//turn to cube
	    	if(!Robot.limelight.hasTarget()) //if robot doesn't see a cube, constantly turn left 
	    	{
	    		Robot.drivetrain.drive(.25, -.25);
	    		return;
	    	}
	    	currentTimeoutLoops = 0;
	    	double area = Robot.limelight.getArea();
	    	double error = Robot.limelight.getX();
	    	double leftOutput = 0;
	    	double rightOutput = 0;
	    	
	    	errorSum += error;
	    	if(Math.abs(error) <= 2) errorSum = 0;
	    	double F = error > 0 ? 0.05 : -0.05;
	    	double P = error * RobotMap.turnLimekP;
	    	double I = errorSum * RobotMap.turnLimekI;
	    	double D = (lastError - error) * (area > 2 ? RobotMap.turnLimekD/5. : RobotMap.turnLimekD);
	    	double output = P + I - D + F;
	    	leftOutput = output;
	    	rightOutput = -output;
	    	
	    	Robot.drivetrain.drive(leftOutput, rightOutput);
	    	lastError = error;
	    	SmartDashboard.putNumber("Vision Error", error);
	    	SmartDashboard.putNumber("Vision OutputL", leftOutput);
	    	SmartDashboard.putNumber("Vision OutputR", rightOutput);
    	} catch(Exception e) {}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(lastError == 0) return false;
    	if(Math.abs(lastError) < 2)
    	{
    		if(settleLoops++ == settleLoopsGoal) return true;
        	else return false;
    	}
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
