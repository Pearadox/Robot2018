package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RotateRight extends Command {

	private double desiredAngle;
	private double originalAngle;
	private double changeInAngle;
	private double speed = .785;
	
    public RotateRight(double angle) {
    	desiredAngle = angle;
    	desiredAngle %= 360;
    }
    
    public RotateRight(double angle, double speed)
    {
    	this(angle);
    	this.speed = speed;
    }


    // Called just before this Command runs the first time
    protected void initialize() {
    	originalAngle = Robot.gyro.getYaw();
    	setTimeout(2);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	changeInAngle = Robot.gyro.getYaw() - originalAngle;
    	Robot.drivetrain.drive(-speed, speed);
    	if(isTimedOut())
    	{
    		if(changeInAngle < 10)
    		{
    			speed += .05;
    			setTimeout(2);
    		}
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	
    		if(changeInAngle >= desiredAngle)
    		{
    			Robot.drivetrain.stop();
    			return true;
    		}
    
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	
    	Robot.drivetrain.drive(0, 0);

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {

    }
}
