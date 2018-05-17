package org.usfirst.frc.team5414.robot.commands;

import java.util.ArrayList;
import java.util.StringTokenizer;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;
//import jaci.pathfinder.Pathfinder;

/**
 * Goes from right side of starting area to the front of the left scale
 * 
 * Approximate Time: 6 sec
 */
public class AutoScaleRtoL extends CommandGroup {

    public AutoScaleRtoL() {

    	addParallel(new ArmPincherClose());
		addParallel(new ZeroGyro());
		addParallel(new ArmSetHover());
		addSequential(new DriveForward(5.45));
		addSequential(new SetAngle(-90));
		addSequential(new SetAngle(-90));
		addSequential(new DriveForward(5.02));
		addSequential(new SetAngle(-160));
		addSequential(new DriveForward(-0.94));
    	addSequential(new ArmThrowbackHigh());
    	addParallel(new ArmSetLow());
    }
}
