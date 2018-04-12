package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *	Goes and turns to cube at same time
 *	CAMERA IS INVERSED
 */
public class VisionGoToCube extends Command {

	final int timeoutLoops = 10; //loops * 20ms = milliseconds of timeout
	final int settleLoopsGoal = 0;
	int settleLoops = 0;
	int currentTimeoutLoops = 0;
	double targetArea = 25;
	double errorSum;
	double lastError;
	double lastAreaError;
	
    public VisionGoToCube() {
    	requires(Robot.drivetrain);
    }

    protected void initialize() {
    	errorSum = 0;
    	lastError = 0;
    	lastAreaError = 0;
    	settleLoops = 0;
    	RobotMap.forwardTurnLimekP = Robot.prefs.getDouble("Limelight Forward Turn kP", RobotMap.forwardTurnLimekP);
    	RobotMap.forwardTurnLimekI = Robot.prefs.getDouble("Limelight Forward Turn kI", RobotMap.forwardTurnLimekI);
    	RobotMap.forwardTurnLimekD = Robot.prefs.getDouble("Limelight Forward Turn kD", RobotMap.forwardTurnLimekD);
    	RobotMap.forwardLimekP = Robot.prefs.getDouble("Limelight Forward kP", RobotMap.forwardLimekP);
    	RobotMap.forwardLimekD = Robot.prefs.getDouble("Limelight Forward kD", RobotMap.forwardLimekD);
    }

    protected void execute() {
    	try {
	    	//turn to cube
	    	if(!Robot.limelight.hasTarget()) //if robot doesn't see a cube, constantly turn left 
	    	{
	    		Robot.drivetrain.drive(.23, -.23);
	    		return;
	    	}
	    	currentTimeoutLoops = 0;
	    	double area = Robot.limelight.getArea();
	    	double error = Robot.limelight.getX();
	    	double leftOutput = 0;
	    	double rightOutput = 0;
	    	
	    	errorSum += error;
	    	if(Math.abs(error) <= 2) errorSum = 0;
	    	double F = error > 0 ? 0.07 : -0.07;
	    	double P = error * RobotMap.forwardTurnLimekP;
	    	double I = errorSum * RobotMap.forwardTurnLimekI;
	    	double D = (lastError - error) * (area > 2 ? RobotMap.turnLimekD/20. : RobotMap.turnLimekD);
	    	double output = P + I - D + F;
	    	leftOutput = output;
	    	rightOutput = -output;
	    	
	    	//Go to cube
	    	double areaError = targetArea - area;
	    	if(areaError < 0) //if already reached or went past cube
	    	{
	    		Robot.drivetrain.stop();
	    		return;
	    	}
	    	double forwardP = areaError * RobotMap.forwardLimekP;
	    	double forwardD = (lastAreaError - areaError) * RobotMap.forwardLimekD;
	    	double forwardOutput = forwardP - forwardD;
	    	leftOutput += forwardOutput;
	    	rightOutput += forwardOutput;
	    	lastAreaError = areaError;
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
        if(lastAreaError < -1)
        {
        	if(settleLoops++ == settleLoopsGoal) return true;
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
