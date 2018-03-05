package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SpintakeOrient extends Command {

	boolean rightIsTriggered = false;
	
    public SpintakeOrient() {
        requires(Robot.spintake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	rightIsTriggered = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
//    	double leftPeakCurrent = 9999;
//    	double rightPeakCurrent = 9999;
    	double leftTriggerRightCurrent = 27; //left triggers right when left current reaches threshold 
    	boolean left = true;
    	boolean right = false;
    	if(Robot.pdp.getLeftSpintake() >= leftTriggerRightCurrent || rightIsTriggered) 
    	{
    		rightIsTriggered = true;
    		right = true;
    	}
    	if(left)
    	{
    		Robot.spintake.setRight(1);
    	}
    	if(right)
    	{
    		Robot.spintake.setLeft(-1);
    	}
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.spintake.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
