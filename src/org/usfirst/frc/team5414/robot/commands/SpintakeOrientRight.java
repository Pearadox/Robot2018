package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SpintakeOrientRight extends Command {

	boolean leftIsTriggered = false;
	
    public SpintakeOrientRight() {
        requires(Robot.spintake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		leftIsTriggered = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(Robot.pdp.getCurrent(12) > 20 || leftIsTriggered)
    	{
    		leftIsTriggered = true;
    		Robot.spintake.intake();
    	}
    	else Robot.spintake.orientRight();
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
