package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SpintakePOVControl extends Command {

	int last = -1;
	
    public SpintakePOVControl() {
    	requires(Robot.spintake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	int opPOV = Robot.oi.getOpJoystick().getPOV();
    	if(opPOV != -1) Robot.spintake.setOrienting(true);
    	switch(opPOV)
    	{
    	case -1: 
    		Robot.spintake.setOrienting(false);
    		Robot.spintake.stopNoBrake(); break;
    	case 315: Robot.spintake.orientLeft();  break;
    	case 0: Robot.spintake.outtake(); break;
    	case 45: Robot.spintake.orientRight(); break;
    	case 90: Robot.spintake.orientRight(); break;
    	case 135: Robot.spintake.orientRight(); break;
    	case 180: Robot.spintake.intake(); break;
    	case 225: Robot.spintake.orientLeft(); break;
    	case 270: Robot.spintake.orientLeft(); break;
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
