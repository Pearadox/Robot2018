
package org.usfirst.frc.team5414.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Goes from right side of starting area to the front of the right scale
 */
public class AutoScaleRtoR extends CommandGroup {

    public AutoScaleRtoR() {
    	addParallel(new ArmPincherClose());
    	addSequential(new Wait(.2));
    	addParallel(new ZeroGyro());
    	addParallel(new ArmSetHover());
    	addSequential(new DriveForward(3));
    	addSequential(new SetAngle(0));
    	addSequential(new DriveForward(3.1));
    	addSequential(new TurnRight(133));
    	addSequential(new DriveForward(-.35));
    	addSequential(new ArmThrowbackHigh());
    	addSequential(new ArmSetLow());
    }
}
