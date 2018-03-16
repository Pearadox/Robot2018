package org.usfirst.frc.team5414.robot.commands;

import java.util.ArrayList;
import java.util.StringTokenizer;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousMidSwitch extends CommandGroup {

    public AutonomousMidSwitch() {
    	try {
    		
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
    	}
    	catch(Exception e)
    	{
    		addSequential(new AutonomousDriveForward());
    	}
    }
}
