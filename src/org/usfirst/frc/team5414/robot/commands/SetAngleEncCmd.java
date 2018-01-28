package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

/**
 *
 */
public class SetAngleEncCmd extends Command {

	double desired = 0;
	double error = 9999;
	
    public SetAngleEncCmd(double desired) {
        this.desired = desired;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double current = Robot.gyro.getTrueYaw();
    	error = desired - current;
    	error %= 360;
    	if(error < 0) //rotate left
    	{
    		double ratio = error / 360;
        	Robot.drivetrain.zeroEncoders();
        	Command cmd = (new DriveEncDist(-(int)Math.round(RobotMap.encodersLeftFullTurn * ratio), (int)Math.round(RobotMap.encodersRightFullTurn * ratio)));
        	Scheduler.getInstance().add(cmd);
        	while(cmd.isRunning()) {System.out.println("RUNNING");} 	
        	System.out.println("LEFT" + " " + -(int)Math.round(RobotMap.encodersLeftFullTurn * ratio) + " " + (int)Math.round(RobotMap.encodersLeftFullTurn * ratio));
    	}
    	else //rotate right
    	{
    		double ratio = error / 360;
        	Robot.drivetrain.zeroEncoders();
        	Command cmd = (new DriveEncDist((int)Math.round(RobotMap.encodersLeftFullTurn * ratio), -(int)Math.round(RobotMap.encodersRightFullTurn * ratio)));
        	Scheduler.getInstance().add(cmd);
        	while(cmd.isRunning()) {System.out.println("RUNNING");} 	
        	System.out.println("Right" + " " + (int)Math.round(RobotMap.encodersLeftFullTurn * ratio) + " " + -(int)Math.round(RobotMap.encodersLeftFullTurn * ratio));
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	System.out.println("ERROR" + error);
        return Math.abs(error) <= 3;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("ENC IS DONE");
    	Robot.drivetrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
