package org.usfirst.frc.team5414.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ArmSetHover extends CommandGroup {

    public ArmSetHover() {
    	addSequential(new ArmSetAngle(80));
    }
}
