package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/*
 * Prioritizes same side switch
 */ 

public class AutonomousSwitchPriorityRight extends CommandGroup {

    public AutonomousSwitchPriorityRight() {

    	char switchSide = Robot.switchSide;
    	char scaleSide = Robot.scaleSide;

    	if(switchSide == 'R') addSequential(new AutoSwitchRtoR());
    	else if(scaleSide == 'R') addSequential(new AutoScaleRtoR());
    	else addSequential(new DriveForward(3));
    }
}
