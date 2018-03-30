package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *	-go to correct side of scale, turn around, and place cube backwards
 *	-use vision to pick up a cube
 *	-go to correct side of scale and place cube
 *	-turn around and use vision to pic k up a cube
 *	- maybe if there's time, try to place it in the switch if it's on same side of scale
 *		-if it's not on same side of scale, put it on the scale
 */
public class AutonomousScalePriorityRight extends CommandGroup {

    public AutonomousScalePriorityRight() {

    	char switchSide = Robot.switchSide;
    	char scaleSide = Robot.scaleSide;
    	    	
    	if(scaleSide == 'R') addSequential(new AutoScaleRtoR());
    	else if(switchSide == 'R') addSequential(new AutoSwitchRtoR());
    	else addSequential(new AutoScaleRtoL());
    }
}
