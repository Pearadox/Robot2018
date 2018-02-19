package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnRight extends Command {

	double halfturn = 535;
	double desired;
	double errorSumLeft = 0;
	double errorSumRight = 0;
	double lastLeftError = 0;
	double lastRightError = 0;
	int settleLoops = 0;
	
    public TurnRight(double degrees) {
    	desired = Math.round(degrees / 180. * halfturn);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	errorSumLeft = 0;
    	errorSumRight = 0;
    	lastLeftError = desired;
    	lastRightError = desired;
    	Robot.drivetrain.zeroEncoders();
    	settleLoops = 5;
    	if(RobotMap.flatbot)
		{
			RobotMap.flatbotkP = Robot.prefs.getDouble("FlatEnc kP", RobotMap.flatbotkP);
			RobotMap.flatbotkI = Robot.prefs.getDouble("FlatEnc kI", RobotMap.flatbotkI);
			RobotMap.flatbotkD = Robot.prefs.getDouble("FlatEnc kD", RobotMap.flatbotkD);
	    	
		}
		else if(!RobotMap.flatbot)
		{
			RobotMap.plybotLkP = Robot.prefs.getDouble("PlyEnc kP", RobotMap.plybotLkP);
			RobotMap.plybotLkI = Robot.prefs.getDouble("PlyEnc kI", RobotMap.plybotLkI);
			RobotMap.plybotLkD = Robot.prefs.getDouble("PlyEnc kD", RobotMap.plybotLkD);
			RobotMap.plybotRkP = Robot.prefs.getDouble("PlyEnc kP", RobotMap.plybotRkP);
			RobotMap.plybotRkI = Robot.prefs.getDouble("PlyEnc kI", RobotMap.plybotRkI);
			RobotMap.plybotRkD = Robot.prefs.getDouble("PlyEnc kD", RobotMap.plybotRkD);
		}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double currentLeft = Robot.drivetrain.getEncoderL();
		double currentRight = Robot.drivetrain.getEncoderR();
		double leftError = desired - currentLeft;
		double rightError = -desired - currentRight;
		errorSumLeft += leftError;
		errorSumRight += rightError;

		//PID Calculations
		double leftP = leftError * (RobotMap.flatbot ? RobotMap.flatbotkP : RobotMap.plybotLkP);
		double rightP = rightError * (RobotMap.flatbot ? RobotMap.flatbotkP : RobotMap.plybotRkP);
		double leftI = errorSumLeft * (RobotMap.flatbot ? RobotMap.flatbotkP : RobotMap.plybotLkI);
		double rightI = errorSumRight * (RobotMap.flatbot ? RobotMap.flatbotkP : RobotMap.plybotRkI);
		double leftD = -(leftError - lastLeftError) * (RobotMap.flatbot ? RobotMap.flatbotkP : RobotMap.plybotLkD); //dSetpoint - dError , prevents derivative kick
		double rightD = -(rightError - lastRightError) * (RobotMap.flatbot ? RobotMap.flatbotkP : RobotMap.plybotRkD); //dSetpoint - dError
		double leftOutput = leftP + leftI - leftD;
		double rightOutput = rightP + rightI - rightD;
		
		Robot.drivetrain.drive(leftOutput, rightOutput);
		
		lastLeftError = leftError;
		lastRightError = rightError;
    }

    protected boolean isFinished() {
    	if(settleLoops == 0)
    	{
    		return true;
    	}
    	else if((lastLeftError) < 5 && Math.abs(lastRightError) < 5)
		{
			settleLoops--;
		}
		return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivetrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
