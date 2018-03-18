package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ArmSetAngle extends Command {

	double desiredAngle;
	double lastError;
	double errorSum;
	int settleLoops;
	final static int settleLoopsGoal = 3;
	boolean accurate = false;
	
	public ArmSetAngle(double angle, boolean accurate) {
        requires(Robot.arm);
        desiredAngle = angle;
        setTimeout(3);
    }
	
	public ArmSetAngle(double angle) {
        this(angle, false);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	errorSum = 0;
    	lastError = (desiredAngle-Robot.arm.getAngle());
    	settleLoops = 0;
    	RobotMap.armkP = Robot.prefs.getDouble("Arm kP", RobotMap.armkP);
		RobotMap.armkI = Robot.prefs.getDouble("Arm kI", RobotMap.armkI);
		RobotMap.armkD = Robot.prefs.getDouble("Arm kD", RobotMap.armkD);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double error = desiredAngle - Robot.arm.getAngle();
    	
    	double F = Robot.arm.calculateHoldOutput(Robot.arm.getAngle());
    	double P = error * RobotMap.armkP;
    	double I = errorSum * RobotMap.armkI;
    	double D = (error-lastError) * RobotMap.armkD;
    	double output = P + I + D + F;
    	
    	if(Math.abs(error) < 1)
    	{
        	output = Robot.arm.calculateHoldOutput(Robot.arm.getAngle());    		
    	}
    	else
    	{
	    	output = Math.max(-.4, output);

    	}
    	Robot.arm.set(output);
    	errorSum += error;
    	lastError = error;
    	SmartDashboard.putNumber("Arm output", output);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
//    	if(settleLoops == settleLoopsGoal) 
//    		return true;
//    	if(Math.abs(lastError) < (accurate ? 1 : 4)) 
//    		settleLoops++;
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.arm.set(Robot.arm.calculateHoldOutput(Robot.arm.getAngle()));
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
