package org.usfirst.frc.team5414.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousSwitchMiddleBackup extends CommandGroup {

    public AutonomousSwitchMiddleBackup() {
    	addSequential(new DriveForward(1));
    	addSequential(new TurnRight(45));
    	addSequential(new DriveForward(4));
    }
}
