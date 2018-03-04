package org.usfirst.frc.team5414.robot;


import java.io.File;

public class PathfinderExample
{

    public static final double DRIVETRAIN_LENGTH = 0.79375;

    public static void main(String[] args)
    {
    	/*
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH,
                0.05, 10, 2.0, 60.0);
        Waypoint[] points = new Waypoint[] {
                new Waypoint(0.0, 0.0, Pathfinder.d2r(0.0)),
                new Waypoint((4.9784 - DRIVETRAIN_LENGTH), 0.0, Pathfinder.d2r(0.0)),
                new Waypoint((4.9784 - DRIVETRAIN_LENGTH) + 2.7, 1.0, Pathfinder.d2r(0.0)),
        };

        Trajectory trajectory = Pathfinder.generate(points, config);

        // Wheelbase Width = 0.5m
        TankModifier modifier = new TankModifier(trajectory).modify(0.5);

        // Do something with the new Trajectories...
        Trajectory left = modifier.getLeftTrajectory();
        Trajectory right = modifier.getRightTrajectory();

        for(Trajectory.Segment s : trajectory.segments)
            System.out.println(s.y);



        //saving trajectory
        File myFile = new File("myfile.traj");
        Pathfinder.writeToFile(myFile, trajectory);

        //reading trajectory
        File myFile2 = new File("myfile.traj");
        trajectory = Pathfinder.readFromFile(myFile2);
        */
    }

}
