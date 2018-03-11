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
public class AutoScaleRtoR extends CommandGroup {

    public AutoScaleRtoR() {
		addSequential(new ZeroGyro());
    	addSequential(new DriveForward(6.8));
    	addSequential(new SetAngle(150));
    	addSequential(new DriveForward(-.5));
    	addSequential(new Wait(3));
    	addSequential(new ArmThrowback());
    }
}