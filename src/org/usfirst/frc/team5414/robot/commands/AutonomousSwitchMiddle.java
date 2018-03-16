package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *	Plan:
 *	-go to correct side of scale, turn around, and place cube backwards
 *	-use vision to pick up a cube
 *	- maybe if there's time, try to place it in the switch if it's on same side of scale
 *		-if it's not on same side of scale, put it on the scale
 */
public class AutonomousSwitchMiddle extends CommandGroup {

    public AutonomousSwitchMiddle() {
    	
    	char switchSide = Robot.switchSide;
    	char scaleSide = Robot.scaleSide;
    	    	
    	if(switchSide == 'L')
    	{
    		addSequential(new AutoSwitchMtoL());
    	}
    	else if(switchSide == 'R')
    	{
    		addSequential(new AutoSwitchMtoR());
    	}
    	else addSequential(new AutonomousSwitchMiddleBackup());
    }
}
