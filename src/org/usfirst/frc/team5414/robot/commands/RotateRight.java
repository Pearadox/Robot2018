package org.usfirst.frc.team5414.robot.commands;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RotateRight extends CommandGroup {

    public RotateRight(double desiredAngle) {
    	double ratio = desiredAngle / 360;
    	addSequential(new ZeroEncoders());
    	addSequential(new DriveEncDist((int)Math.round(RobotMap.encodersLeftFullTurn * ratio), -(int)Math.round(RobotMap.encodersRightFullTurn * ratio)));
    }
}
