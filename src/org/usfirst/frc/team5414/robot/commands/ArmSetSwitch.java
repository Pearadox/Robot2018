package org.usfirst.frc.team5414.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ArmSetSwitch extends CommandGroup {

    public ArmSetSwitch() {
    	addSequential(new ArmSetAngle(90));
    }
}
