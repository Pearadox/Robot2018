package org.usfirst.frc.team5414.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class SpintakeIntakeCommandDelayed extends CommandGroup {

    public SpintakeIntakeCommandDelayed(double delay) {
    	addSequential(new Wait(delay));
    	addSequential(new SpintakeIntakeCommand());
    }
}
