package org.usfirst.frc.team5414.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ArmThrowbackHighGroup extends CommandGroup {

    public ArmThrowbackHighGroup() {
        addSequential(new SpintakePushOut());
        addSequential(new Wait(.4)); //change this to whatever, it's in seconds
        addSequential(new ArmThrowbackHigh());
    }
}
