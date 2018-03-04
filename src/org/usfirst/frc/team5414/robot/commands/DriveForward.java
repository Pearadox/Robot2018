package org.usfirst.frc.team5414.robot.commands;

import java.util.ArrayList;

import org.usfirst.frc.team5414.robot.RobotMap;
import org.usfirst.frc.team5414.robot.TrajectoryGenerator;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *	Drivers forward/backwards an accurate amount of distance
 *	To go backwards, set the feet to negative
 */
public class DriveForward extends CommandGroup {

	double maxVelocity = 15;
	double acceleration = 5;
	
//	Generates a trapezoidal trajectory and follows the encoders
    public DriveForward(double feet) {
        org.usfirst.frc.team5414.robot.Traj[] left = TrajectoryGenerator.getTrajectory(feet, .02, maxVelocity, acceleration);
        org.usfirst.frc.team5414.robot.Traj[] right = TrajectoryGenerator.getTrajectory(feet, .02, maxVelocity, acceleration);
        ArrayList<Double> leftList = new ArrayList<>();
        ArrayList<Double> rightList = new ArrayList<>();
        int skip = 0;
        for(int i = 0; i < left.length; i+= 1 + skip) 
        {
        	leftList.add(left[i].distance / RobotMap.FeetPerTick);
        	rightList.add(right[i].distance / RobotMap.FeetPerTick);
       	}
        addSequential(new FollowEncoder(leftList, rightList));
    }
}
