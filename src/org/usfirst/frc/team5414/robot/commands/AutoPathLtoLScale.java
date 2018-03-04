package org.usfirst.frc.team5414.robot.commands;

import java.util.ArrayList;
import java.util.StringTokenizer;

import org.usfirst.frc.team5414.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;
//import jaci.pathfinder.Pathfinder;

/**
 * Goes from left side of starting area to the front of the right scale
 */
public class AutoPathLtoLScale extends CommandGroup {

    public AutoPathLtoLScale() {
    	
    	StringTokenizer st = null;
    	ArrayList<Double> left = new ArrayList<>();
    	ArrayList<Double> right = new ArrayList<>();
    	String path = "";
    	int skip = 4;
    	int count = 0;
    	
    	int startingKickSkip = 100;
    	st = new StringTokenizer(path);
    	
    	try {	
    		for(int i = 0; i < startingKickSkip; i++) 
    		{
    			st.nextToken();
    			st.nextToken();
    		}
    		while(st.hasMoreTokens())
        	{
    			if(count++ == skip)
    			{
					double r = Double.parseDouble(st.nextToken());
					double l = Double.parseDouble(st.nextToken());
					left.add(l);
					right.add(r);
					count = 0;
    			}
    			else
    			{
    				try {
    				st.nextToken();
    				st.nextToken();
    				} catch(Exception e) {}
    			}
        	}
    	} catch(Exception e) {
    		e.printStackTrace();
    		Robot.drivetrain.stop();
    		DriverStation.reportWarning("NOOOO", true);
    	}
		addSequential(new ZeroGyro());
    	addSequential(new FollowEncoder(left, right));
    	addSequential(new SetAngle(30));
    }
}
