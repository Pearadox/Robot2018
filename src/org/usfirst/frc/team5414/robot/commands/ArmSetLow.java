package org.usfirst.frc.team5414.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ArmSetLow extends CommandGroup {

    public ArmSetLow() {
    	addSequential(new ArmSetAngle(50-4));
    }
}
