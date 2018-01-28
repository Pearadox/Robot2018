package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class SetAngleEnc extends CommandGroup {

    public SetAngleEnc(double desired) {
    	double error = 1000;
    	while(error >= 3)
    	{
	    	
	    	error %= 360;
	    	if(error < 0) //rotate left
	    	{
	    		System.out.println("LEFT");
	    		addSequential(new RotateLeft(error));
	    	}
	    	else //rotate right
	    	{
	    		System.out.println("RIGHT");
	    		addSequential(new RotateRight(error));
	    	}
    	}
    }
}
