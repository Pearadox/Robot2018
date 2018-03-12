package org.usfirst.frc.team5414.robot.commands;

import java.util.ArrayList;
import java.util.StringTokenizer;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.command.CommandGroup;

/*
 * Prioritizes same side scale
 */
public class AutonomousScalePriorityLeft extends CommandGroup {

    public AutonomousScalePriorityLeft() {

    	String gameData = DriverStation.getInstance().getGameSpecificMessage();
    	char switchSide = gameData.charAt(0); // 'L' or 'R'
    	char scaleSide = gameData.charAt(1); // 'L' or 'R'

    	if(scaleSide == 'L') addSequential(new AutoScaleLtoL());
    	else if(switchSide == 'L') addSequential(new AutoSwitchLtoL());
    	else addSequential(new AutoScaleLtoR());
    }
}
