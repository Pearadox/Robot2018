/*----------------------------------------------------------------------------
	Team 5414 Pearadox 2018 Build Season Code
	Heckled by 118
  ----------------------------------------------------------------------------*/

package org.usfirst.frc.team5414.robot;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

import org.usfirst.frc.team5414.robot.commands.AutonomousScaleLeft;
import org.usfirst.frc.team5414.robot.subsystems.Drivetrain;
import org.usfirst.frc.team5414.robot.subsystems.IMU;

public class Robot extends TimedRobot {
	
	@SuppressWarnings("deprecation")
	NetworkTable table = NetworkTable.getTable("limelight");
	public static Drivetrain drivetrain;
	public static OI oi;
	public static IMU gyro; 
	
	Command autonomousCommand;
	
	@Override
	public void robotInit() {
		oi = new OI();
		drivetrain = new Drivetrain();
//		gyro = new IMU();
		
//		gyro.initialize();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		table.putNumber("ledMode", 1);
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		autonomousCommand = new AutonomousScaleLeft();
		if (autonomousCommand != null) {
			autonomousCommand.start();
		}
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
		table.putNumber("ledMode", 2);
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void testPeriodic() {
	}
}
