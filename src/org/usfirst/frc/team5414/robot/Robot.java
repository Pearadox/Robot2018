/*----------------------------------------------------------------------------
	Team 5414 Pearadox 2018 Build Season Code
	Heckled by 118
  ----------------------------------------------------------------------------*/

package org.usfirst.frc.team5414.robot;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.PrintWriter;

import org.usfirst.frc.team5414.robot.commands.AutonomousLeftToRightScale;
import org.usfirst.frc.team5414.robot.commands.AutonomousScaleLeft;
import org.usfirst.frc.team5414.robot.commands.DriveEncDist;
import org.usfirst.frc.team5414.robot.subsystems.Drivetrain;
import org.usfirst.frc.team5414.robot.subsystems.IMU;

public class Robot extends TimedRobot {
	
//	NetworkTable table = NetworkTable.getTable("limelight");
	public static Drivetrain drivetrain;
	public static OI oi;
	public static IMU gyro; 
	public static Compressor compressor;
	public static Preferences prefs;
	
	Command autonomousCommand;
	
	@Override
	public void robotInit() {
		drivetrain = new Drivetrain();
		oi = new OI();
//		compressor = new Compressor(0);
//		gyro = new IMU();
		
//		compressor.start();
//		gyro.initialize();
		prefs.putDouble("Enc kP", RobotMap.kP);
		prefs.putDouble("Enc kI", RobotMap.kI);
		prefs.putDouble("Enc kD", RobotMap.kD);
		prefs.putInt("Desired Left Enc", 0);
		prefs.putInt("Desired Right Enc", 0);
		SmartDashboard.putData("Test Drive Enc", new DriveEncDist(prefs.getInt("Desired Left Enc", 0), prefs.getInt("Desired Right Enc", 0)));
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		autonomousCommand = new AutonomousLeftToRightScale();
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
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void testPeriodic() {
	}
}
