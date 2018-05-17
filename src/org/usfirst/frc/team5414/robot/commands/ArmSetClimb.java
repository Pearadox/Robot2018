package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ArmSetClimb extends CommandGroup {

	 	
    public ArmSetClimb() {
    	addParallel(new SpintakePushOut());
    	addSequential(new ArmSetAngle(162));
    }
}
