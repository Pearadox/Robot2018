package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ArmSetScale extends CommandGroup {

    public ArmSetScale() {
    	addSequential(new ArmSetAngle(180));
    }
}
