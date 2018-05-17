package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *	both sides work as of 5/16
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
