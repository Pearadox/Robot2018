package org.usfirst.frc.team5414.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/*
 * Prioritizes same side switch
 */

public class AutonomousSwitchPriorityRight extends CommandGroup {

    public AutonomousSwitchPriorityRight() {

    	String gameData = DriverStation.getInstance().getGameSpecificMessage();
    	char switchSide = gameData.charAt(0); // 'L' or 'R'
    	char scaleSide = gameData.charAt(1); // 'L' or 'R'

    	if(switchSide == 'R') addSequential(new AutoSwitchRtoR());
    	else if(scaleSide == 'R') addSequential(new AutoScaleRtoR());
    	else addSequential(new AutoScaleRtoL());
    }
}
