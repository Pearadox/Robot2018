package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SpintakeOrientLeft extends Command {

	boolean rightIsTriggered = false;
	
    public SpintakeOrientLeft() {
        requires(Robot.spintake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.spintake.orientLeft();
    	rightIsTriggered = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		if(Robot.pdp.getCurrent(3) > 20 || rightIsTriggered)
    	{
    		rightIsTriggered = true;
    		Robot.spintake.intake();// this isnt right because we need
    		//make it so that it doesnt continuously run this in the executve f
    	}
    	else Robot.spintake.orientLeft();    	
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
