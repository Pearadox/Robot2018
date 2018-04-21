package org.usfirst.frc.team5414.robot.commands;

import java.util.ArrayList;

import org.usfirst.frc.team5414.robot.RobotMap;
import org.usfirst.frc.team5414.robot.TrajectoryGenerator;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *	Turns the robot to the relative left the specified amount of degress
 */
public class TurnLeft extends CommandGroup {

	double halfturn = 256;
	final static double maxVelocity_default = 3;
	final static double acceleration_default = 2.5;
	
    public TurnLeft(double degrees, double maxVelocity, double acceleration) {
        double desired = degrees / 180. * halfturn;
        org.usfirst.frc.team5414.robot.Traj[] left = TrajectoryGenerator.getTrajectory(-desired*RobotMap.MetersPerTick, .02, maxVelocity, acceleration);
        org.usfirst.frc.team5414.robot.Traj[] right = TrajectoryGenerator.getTrajectory(desired*RobotMap.MetersPerTick, .02, maxVelocity, acceleration);
        ArrayList<Double> leftList = new ArrayList<>();
        ArrayList<Double> rightList = new ArrayList<>();
        int skip = 0;
        for(int i = 0; i < left.length; i+= 1 + skip) 
        {
        	leftList.add(left[i].distance / RobotMap.MetersPerTick);
        	rightList.add(right[i].distance / RobotMap.MetersPerTick);
       	}
        addSequential(new FollowEncoder(leftList, rightList));
    }
    
    public TurnLeft(double degrees)
    {
    	this(degrees, maxVelocity_default, acceleration_default);
    }
}
