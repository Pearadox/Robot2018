package org.usfirst.frc.team5414.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *	Plan:
 *	- go to correct side of scale and place a cube
  *	-turn around and use vision to pick up a cube
 *	- maybe if there's time, try to place it in the switch if it's on same side of scale
 *		-if it's not on same side of scale, put it on the scale
 */
public class AutonomousScaleMiddle extends CommandGroup {

    public AutonomousScaleMiddle() {
    	
    	String gameData = DriverStation.getInstance().getGameSpecificMessage();
    	char switchSide = gameData.charAt(0); // 'L' or 'R'
    	char scaleSide = gameData.charAt(1); // 'L' or 'R'
    	
    	if(scaleSide == 'L')
    	{
    		addSequential(new AutoPathMtoLScale());
    	}
    	else if(scaleSide == 'R')
    	{
    		addSequential(new AutoPathMtoRScale());
    	}
    }
}
