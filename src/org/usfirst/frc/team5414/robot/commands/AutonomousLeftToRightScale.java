package org.usfirst.frc.team5414.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
//import jaci.pathfinder.Pathfinder;
//import jaci.pathfinder.Trajectory;
//import jaci.pathfinder.Waypoint;

/**
 * Goes from left side of starting area to the front of the right scale
 */
public class AutonomousLeftToRightScale extends CommandGroup {

    public AutonomousLeftToRightScale() {
    	
//		Waypoint[] points = new Waypoint[] {
//				new Waypoint(0,0,0),
//				new Waypoint(1,1,0)
//				
//			};

			// Create the Trajectory Configuration
			//
			// Arguments:
			// Fit Method:          HERMITE_CUBIC or HERMITE_QUINTIC
			// Sample Count:        SAMPLES_HIGH (100 000)
//		   	                      SAMPLES_LOW  (10 000)
//			                      SAMPLES_FAST (1 000)
			// Time Step:           0.05 Seconds
			// Max Velocity:        1.7 m/s
			// Max Acceleration:    2.0 m/s/s
			// Max Jerk:            60.0 m/s/s/s
//			Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
//
//			// Generate the trajectory
//			Trajectory trajectory = Pathfinder.generate(points, config);
//			addSequential(new DriveEncDist(trajectory));
    }
}
