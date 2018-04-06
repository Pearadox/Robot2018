
package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

/**
 *
 */
public class DrivewithJoystick extends Command {

    public DrivewithJoystick() {
        requires(Robot.drivetrain);   
    }
  
    protected void initialize() {
    }

   
    protected void execute() {
    	Joystick js = Robot.oi.getJoystick();
    	boolean cheesy = false;
    	boolean peary = true;
    	if(cheesy)
    	{
    		if(RobotMap.flatbot) Robot.drivetrain.cheesyDrive(js.getRawAxis(1), -js.getRawAxis(2)*.5, false);
    		if(RobotMap.compbot) Robot.drivetrain.cheesyDrive(-js.getRawAxis(1), js.getRawAxis(2)*.5, false); //changed angle factor
    	}
    	else if(peary) {
    		Robot.drivetrain.pearDrive(-js.getRawAxis(1), -js.getRawAxis(2)*.8);
    	}
    	else Robot.drivetrain.arcadeDrive(Robot.oi.getJoystick());
    	
    	
    	int pov = js.getPOV();
    	if(pov != -1)
    	{
    		Scheduler.getInstance().add(new SetAngle(pov));
    	}
    }

   
    protected boolean isFinished() {
        return false;
    }

   
    protected void end() {
    	Robot.drivetrain.stop();		//sets all motorspeeds to 0
    }
   

    protected void interrupted() {
    	end();
    }
}
