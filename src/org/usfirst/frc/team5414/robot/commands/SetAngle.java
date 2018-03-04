package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *	Turns the robot to the absolute angle specified (compared to relative inside TurnLeft and TurnRight commands)
 *	This will always turn in the quickest way possible
 *	You can pretty much use any possible desired angle (negative, positive, greater than 360), the program will fix the angle for you :) 
 */
public class SetAngle extends CommandGroup {

    public SetAngle(double desired) {
    	double current = (Robot.gyro.getYaw()%360+360)%360;
    	desired += 360000;
    	desired %= 360;
    	double error = Math.abs(current-desired);
    	System.out.println(desired);
    	if(error >= 180)
    	{
    		error = 360 - error;
    		if(current > desired)
    		{
    			addSequential(new TurnRight(error));
    		}
    		else
    		{
    			addSequential(new TurnLeft(error));
    		}
    	}
    	else
    	{
    		if(current > desired)
    		{
    			addSequential(new TurnLeft(error));
    		}
    		else
    		{
    			addSequential(new TurnRight(error));
    		}
    	}
    }
}
