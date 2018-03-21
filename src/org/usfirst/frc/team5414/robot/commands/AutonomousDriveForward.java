package org.usfirst.frc.team5414.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousDriveForward extends CommandGroup {

    public AutonomousDriveForward() {
    	addSequential(new DriveForward(3));
    }
}
