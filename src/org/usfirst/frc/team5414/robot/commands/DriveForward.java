package org.usfirst.frc.team5414.robot.commands;

import java.util.ArrayList;

import org.usfirst.frc.team5414.robot.RobotMap;
import org.usfirst.frc.team5414.robot.TrajectoryGenerator;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *	Drivers forward/backwards an accurate amount of distance
 *	To go backwards, set the meters to negative
 */
public class DriveForward extends CommandGroup {

	static double maxVelocity = 4.5;
	static double acceleration = 2.8;
	
//	Generates a trapezoidal trajectory and follows the encoders
	public DriveForward(double meters) {
        this(meters, 9999);
    }
	
	public DriveForward(double meters, double timeout)
	{
		this(meters, timeout, maxVelocity, acceleration);
	}
	
	public DriveForward(double meters, double timeout, double maxVelocity, double acceleration)
	{
		org.usfirst.frc.team5414.robot.Traj[] left = TrajectoryGenerator.getTrajectory(meters, .02, maxVelocity, acceleration);
        org.usfirst.frc.team5414.robot.Traj[] right = TrajectoryGenerator.getTrajectory(meters, .02, maxVelocity, acceleration);
        ArrayList<Double> leftList = new ArrayList<>();
        ArrayList<Double> rightList = new ArrayList<>();
        int skip = 0;
        for(int i = 0; i < left.length; i+= 1 + skip) 
        {
        	leftList.add(left[i].distance / RobotMap.MetersPerTick);
        	rightList.add(right[i].distance / RobotMap.MetersPerTick);
       	}
        addSequential(new FollowEncoder(leftList, rightList, timeout));
	}
}
