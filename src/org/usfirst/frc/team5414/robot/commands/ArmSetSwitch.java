package org.usfirst.frc.team5414.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ArmSetSwitch extends CommandGroup {

	public ArmSetSwitch() {
    	this(0);
    }
	
	public ArmSetSwitch(double delay) {
		if(delay != 0) addSequential(new Wait(delay));
    	addSequential(new ArmSetAngle(105));
    }
}
