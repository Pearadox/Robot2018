package org.usfirst.frc.team5414.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
//import jaci.pathfinder.Pathfinder;

/**
 * Goes from left side of starting area to the front of the left scale
 * 
 * 	Approximate Time: 4 sec
 */
public class AutoScaleLtoL extends CommandGroup {

    public AutoScaleLtoL() {
    	
    	addParallel(new ArmPincherClose());
    	addSequential(new Wait(.3));
    	addSequential(new ZeroGyro());
    	addSequential(new DriveForward(7));
    	addSequential(new SetAngle(-150));
    	addSequential(new DriveForward(-.3));
    	addSequential(new ArmThrowbackHigh());
    }
}
