package org.usfirst.frc.team5414.robot.commands;

import java.util.ArrayList;
import java.util.StringTokenizer;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;
//import jaci.pathfinder.Pathfinder;

/**
 * Goes from left side of starting area to the front of the left scale
 * 
 * 	Approximate Time: 4 sec
 */
public class AutoScaleLtoL extends CommandGroup {

    public AutoScaleLtoL() {
		addSequential(new ZeroGyro());
    	addSequential(new DriveForward(7.2));
    	addSequential(new SetAngle(-150));
    	addSequential(new Wait(3));
    	addSequential(new DriveForward(0));
    	addSequential(new ArmThrowback());
    }
}