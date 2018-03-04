package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class MotionMagic extends Command {

	int left, right;
	
    public MotionMagic(int leftTarget, int rightTarget) {
        requires(Robot.drivetrain);
        left = leftTarget;
        right = rightTarget;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drivetrain.setPIDLeft(Robot.prefs.getDouble("Motion Magic Left kF", RobotMap.MMLeftkF), 
    			Robot.prefs.getDouble("Motion Magic Left kP", RobotMap.MMLeftkP), 
    			Robot.prefs.getDouble("Motion Magic Left kI", RobotMap.MMLeftkI), 
    			Robot.prefs.getDouble("Motion Magic Left kD", RobotMap.MMLeftkD));
    	Robot.drivetrain.motionMagic(left, right);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	System.out.println("ISFINISHED()");
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
