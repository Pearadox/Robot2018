package org.usfirst.frc.team5414.robot.commands;

//import javax.swing.plaf.synth.SynthSeparatorUI;

//import jaci.pathfinder.*;
//import jaci.pathfinder.modifiers.TankModifier;

public class test {

//	public static void main(String[] args) {
//
//		// 3 Waypoints
//		Waypoint[] points = new Waypoint[] {
//		    new Waypoint(0, 0, 0),     
//		    new Waypoint(-1, 220, 90),
//		    new Waypoint(150, 221, 0)
//		};
//
//		// Create the Trajectory Configuration
//		//
//		// Arguments:
//		// Fit Method:          HERMITE_CUBIC or HERMITE_QUINTIC
//		// Sample Count:        SAMPLES_HIGH (100 000)
////	   	                      SAMPLES_LOW  (10 000)
////		                      SAMPLES_FAST (1 000)
//		// Time Step:           0.05 Seconds
//		// Max Velocity:        1.7 m/s
//		// Max Acceleration:    2.0 m/s/s
//		// Max Jerk:            60.0 m/s/s/s
//		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_FAST, 0.05, 1.7, 2.0, 60.0);
//		System.out.println("a");
//		// Generate the trajectory
//		Trajectory trajectory = Pathfinder.generate(points, config);
//		System.out.println("b");
//		// The distance between the left and right sides of the wheelbase is 0.6m
//		double wheelbase_width = 0.6;
//
//		// Create the Modifier Object
//		TankModifier modifier = new TankModifier(trajectory);
//
//		// Generate the Left and Right trajectories using the original trajectory
//		// as the centre
//		modifier.modify(wheelbase_width);
//
//		Trajectory left  = modifier.getLeftTrajectory();       // Get the Left Side
//		Trajectory right = modifier.getRightTrajectory();      // Get the Right Side
//		
//		for (int i = 0; i < trajectory.length(); i+=10) {
//		    Trajectory.Segment segLeft = left.get(i);
//		    
//		    System.out.printf("%f,%f,%f\n", 
//		        segLeft.x, segLeft.y, segLeft.velocity);
//		    
//		    Trajectory.Segment segRight = right.get(i);
//		    
//		    System.out.printf("%f,%f,%f\n", 
//		    		segRight.x, segRight.y, segRight.velocity, segRight.jerk);
//		    System.out.println("---------");
//		}
//	}

}
