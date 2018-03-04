package org.usfirst.frc.team5414.robot.commands;

import java.util.ArrayList;

import org.usfirst.frc.team5414.robot.RobotMap;
import org.usfirst.frc.team5414.robot.TrajectoryGenerator;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DriveForward extends CommandGroup {

	double maxVelocity = 15;
	double acceleration = 5;
	
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
