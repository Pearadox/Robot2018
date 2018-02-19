package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class SetAngle extends CommandGroup {

    public SetAngle(double desired) {
    	double current = Robot.gyro.getTrueYaw();
    	double error = Math.abs(current-desired);
    	if(error >= 180)
    	{
    		error -= 180;
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
    			addSequential(new TurnRight(error));
    		}
    		else
    		{
    			addSequential(new TurnLeft(error));
    		}
    	}
    }
}
