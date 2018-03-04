package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class SetAngle extends CommandGroup {

    public SetAngle(double desired) {
    	double current = (Robot.gyro.getYaw()%360+360)%360;
    	desired += 3600;
    	desired %= 360;
    	double error = Math.abs(current-desired);
    	System.out.println(desired);
    	if(error >= 180)
    	{
    		error = 360 - error;
    		if(current > desired)
    		{
    			System.out.println("RIGHT " + error);
    			addSequential(new TurnRight(error));
    		}
    		else
    		{
    			System.out.println("LEFT " + error);
    			addSequential(new TurnLeft(error));
    		}
    	}
    	else
    	{
    		if(current > desired)
    		{
    			System.out.println("LEFTT " + error);
    			addSequential(new TurnLeft(error));
    		}
    		else
    		{
    			System.out.println("RIGHTT " + error);
    			addSequential(new TurnRight(error));
    		}
    	}
    }
}
