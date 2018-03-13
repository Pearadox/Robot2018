package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

/**
 *
 */
public class SetAngle extends Command {

	double desired;
	
    public SetAngle(double desired) {
    	this.desired = desired;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	double current = (Robot.gyro.getYaw()+360)%360;
    	desired += 360000;
    	desired %= 360;
    	double error = Math.abs(current-desired);
    	Robot.turnIsDone = false;
    	if(error >= 180)
    	{
    		error = 360 - error;
    		if(current > desired)
    		{
    			Scheduler.getInstance().add(new TurnRight(error));
    		}
    		else
    		{
    			Scheduler.getInstance().add(new TurnLeft(error));
    		}
    	}
    	else
    	{
    		if(current > desired)
    		{
    			Scheduler.getInstance().add(new TurnLeft(error));
    		}
    		else
    		{
    			Scheduler.getInstance().add(new TurnRight(error));
    		}
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.turnIsDone;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
