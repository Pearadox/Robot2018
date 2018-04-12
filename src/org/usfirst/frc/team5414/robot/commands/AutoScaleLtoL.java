
package org.usfirst.frc.team5414.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Goes from left side of starting area to the front of the left scale
 */
public class AutoScaleLtoL extends CommandGroup {

    public AutoScaleLtoL() {
    	addParallel(new ArmPincherClose());
    	addSequential(new Wait(.2));
    	addParallel(new ZeroGyro());
    	addParallel(new ArmSetHover());
//    	addSequential(new DriveForward(3));
//    	addSequential(new SetAngle(0));
//    	addSequential(new DriveForward(3.1));
    	addSequential(new DriveForward(6.1));
    	addSequential(new TurnLeft(133));
    	addSequential(new DriveForward(-.35));
    	addSequential(new ArmThrowbackHigh());
    	addParallel(new ArmSetLow());
    	addSequential(new SetAngle(148));
    	addSequential(new DriveForward(1.35));
    	addSequential(new ArmPincherClose());
    	addSequential(new DriveForward(-.3));
    	addSequential(new SetAngle(-170));
    	addSequential(new DriveForward(-1.));
    	addSequential(new ArmThrowbackLow());
    	addParallel(new ArmSetLow());
    }
}
