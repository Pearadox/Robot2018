package org.usfirst.frc.team5414.robot.paths;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

import static org.usfirst.frc.team5414.robot.paths.AutoPathsConstants.DRIVETRAIN_LENGTH;

public class LeftSideCloseSwitchPaths {

    public static Waypoint[] toSwitch = new Waypoint[]{
            new Waypoint(0.0, 0.0, Pathfinder.d2r(0.0)),
            new Waypoint((3.55 - DRIVETRAIN_LENGTH), -1, Pathfinder.d2r(0.0)),
    };

}
