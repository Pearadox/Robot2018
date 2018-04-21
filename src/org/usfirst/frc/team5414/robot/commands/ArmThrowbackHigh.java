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
        desiredAngle = 148;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	setTimeout(1.5);
    	Robot.spintake.pushOut();
    	errorSum = 0;
    	lastError = (desiredAngle-Robot.arm.getAngle());
    	settleLoops = 0;
    	RobotMap.armThrowHighkP = Robot.prefs.getDouble("Arm Throw High kP", RobotMap.armThrowHighkP);
		RobotMap.armThrowHighkD = Robot.prefs.getDouble("Arm Throw High kD", RobotMap.armThrowHighkD);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    	if(Robot.arm.getAngle() >= desiredAngle-10) Robot.arm.openPincher();
    	
    	
    	double error = desiredAngle - Robot.arm.getAngle();
    	double F = Robot.arm.calculateHoldOutput(Robot.arm.getAngle());
    	double P = error * RobotMap.armThrowHighkP;
    	double D = (error-lastError) * RobotMap.armThrowHighkD;
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
