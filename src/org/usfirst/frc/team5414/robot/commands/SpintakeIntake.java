package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SpintakeIntake extends Command {

    public SpintakeIntake() {
//    	requires(Robot.spintake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.spintake.intake();
    }

    // Make this return true when this C 
 //   Command no longer needs to run execute();
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.spintake.stop();
    }

    // Called whe n another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
