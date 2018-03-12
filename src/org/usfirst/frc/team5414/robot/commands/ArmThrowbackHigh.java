package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ArmThrowbackHigh extends Command {

	double desiredAngle;
	double lastError;
	double errorSum;
	int settleLoops;
	final static int settleLoopsGoal = 2;
	
    public ArmThrowbackHigh() {
        requires(Robot.arm);
        desiredAngle = 145;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	setTimeout(1.5);
    	errorSum = 0;
    	lastError = (desiredAngle-Robot.arm.getAngle());
    	settleLoops = 0;
    	RobotMap.armkP = Robot.prefs.getDouble("Arm Throw kP", RobotMap.armThrowkP);
		RobotMap.armkD = Robot.prefs.getDouble("Arm Throw kD", RobotMap.armThrowkD);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	if(Robot.arm.getAngle() >= desiredAngle-10) Robot.arm.openPincher();
    	
    	double error = desiredAngle - Robot.arm.getAngle();
    	double F = Robot.arm.calculateHoldOutput(Robot.arm.getAngle());
    	double P = error * RobotMap.armThrowkP;
    	double D = (error-lastError) * RobotMap.armThrowkD;
    	double output = P + D + F;
    	output = Math.max(-.4, output);
    	Robot.arm.set(output);
    	errorSum += error;
    	lastError = error;
    	SmartDashboard.putNumber("Arm output", output);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return lastError < 0 || isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.arm.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
