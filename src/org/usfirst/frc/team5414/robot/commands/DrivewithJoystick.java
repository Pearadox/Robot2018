package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

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
    	Robot.drivetrain.cheesyDrive(js.getRawAxis(1), -js.getRawAxis(2)*1, false, false);
//    	Robot.drivetrain.arcadeDrive(Robot.oi.getJoystick());
//    	Robot.drivetrain.arcadeDrive(-Robot.oi.getJoystick().getY(), Robot.oi.getJoystick().getZ());
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
