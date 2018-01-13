package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class RotateLeft extends Command {

	private double desiredAngle;
	private double changeInAngle;
	private double originalAngle;
	private double speed = .785;
	
    public RotateLeft(double angle) {
    	desiredAngle = angle;
    	desiredAngle %= 360;
    }
    
    public RotateLeft(double angle, double speed)
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
    	Robot.drivetrain.drive(speed, -speed);
    	if(isTimedOut())
    	{
    		if(changeInAngle < 10)
    		{
    			speed += .05;
    			setTimeout(2);
    		}
    	}
    	SmartDashboard.putNumber("changeInAngle", changeInAngle);
    	SmartDashboard.putNumber("desiredAngle", desiredAngle);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(Math.abs(changeInAngle) >= desiredAngle)
		{
			Robot.drivetrain.stop();
			return true;
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
    	Robot.drivetrain.stop();
    }
}
