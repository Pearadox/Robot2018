package org.usfirst.frc.team5414.robot.paths;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

public class GenericPaths {

    public static Waypoint[] path = new Waypoint[]{
            new Waypoint(0.0, 0.0, Pathfinder.d2r(0.0)),
            new Waypoint(1.0, 1.0, Pathfinder.d2r(90.0)),
    };


}
