package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class SetAngle extends Command {

	double desired;
	double errorSum = 0;
	double lastError = 4;
	
	//0 - 359
    public SetAngle(double angle) {
        requires(Robot.drivetrain);
        angle %= 360;
        desired = angle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	RobotMap.gykP = Robot.prefs.getDouble("Gyro kP", RobotMap.gykP);
		RobotMap.gykI = Robot.prefs.getDouble("Gyro kI", RobotMap.gykI);
		RobotMap.gykD = Robot.prefs.getDouble("Gyro kD", RobotMap.gykD);
		System.out.println(RobotMap.gykD);
		errorSum = 0;
		lastError = 0;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double current = Robot.gyro.getTrueYaw()%360;
    	current += 360;
    	current %= 360;
    	if(current >= 180)
    	{
    		current -= 360;
    	}
    	double error = desired - current;
    	error %= 180;
    	double P = error * RobotMap.gykP;
    	double I = errorSum * RobotMap.gykI;
    	double D = (error - lastError) * RobotMap.gykD;
    	double output = P + I - D;
    	if(error >= 0)
    	{
    		if(error > 180)
    		{
    			Robot.drivetrain.drive(output, -output);
    		}
    		else
    		{
    			Robot.drivetrain.drive(-output, output);
    		}
    	}
    	else
    	{
    		if(error < -180)
    		{
    			Robot.drivetrain.drive(output, -output);
    		}
    		else
    		{
    			Robot.drivetrain.drive(-output, output);
    		}
    	}
    	errorSum += error;
    	lastError = error;
    	SmartDashboard.putNumber("Gyro PID Output", output);
    	SmartDashboard.putNumber("Gyro Error", error);
    	SmartDashboard.putNumber("Gyro Current", current);
    	SmartDashboard.putNumber("Gyro Error Sum", errorSum);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if(Math.abs(lastError) <3)
        {
        	if(lastError < 0)
        	{
        		Robot.drivetrain.drive(-1, 1);
        		try {
					Thread.sleep(250);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
        		Robot.drivetrain.stop();
        	}
        	else
        	{
        		Robot.drivetrain.drive(1, -1);
        		try {
					Thread.sleep(250);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
        		Robot.drivetrain.stop();
        	}
        	return true;
        }
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println(lastError);
    	lastError = 1000;
    	Robot.drivetrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
