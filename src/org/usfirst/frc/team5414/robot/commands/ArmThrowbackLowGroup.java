package org.usfirst.frc.team5414.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ArmThrowbackLowGroup extends CommandGroup {

    public ArmThrowbackLowGroup() {
        addSequential(new SpintakePushOut());
        addSequential(new Wait(.2)); //change this to whatever, it's in seconds
        addSequential(new ArmThrowbackLow());
    }
}
