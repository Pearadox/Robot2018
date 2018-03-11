package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;

/**
 *
 */
public class FollowPath extends Command {

	EncoderFollower[] followers;
	boolean reverse;
	
    public FollowPath(Waypoint[] waypoints, boolean reverse) {
    	requires(Robot.drivetrain);
    	RobotMap.MPkP = Robot.prefs.getDouble("Motion Profile kP", RobotMap.MPkP);
    	RobotMap.MPkI = Robot.prefs.getDouble("Motion Profile kI", RobotMap.MPkI);
    	RobotMap.MPkD = Robot.prefs.getDouble("Motion Profile kD", RobotMap.MPkD);
    	followers = Robot.drivetrain.pathSetup(waypoints);
    	this.reverse = reverse;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.drivetrain.pathFollow(followers, reverse);
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
