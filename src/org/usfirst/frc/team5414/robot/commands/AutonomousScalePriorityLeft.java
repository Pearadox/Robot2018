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

    	char switchSide = Robot.switchSide;
    	char scaleSide = Robot.scaleSide;
    	
    	if(scaleSide == 'L') addSequential(new AutoScaleSwitchLtoL());
    	else if(switchSide == 'L') addSequential(new AutoSwitchLtoL());
    	else addSequential(new AutoScaleLtoR());
    }
}
