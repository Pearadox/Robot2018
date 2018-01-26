/*----------------------------------------------------------------------------
	Team 5414 Pearadox 2018 Build Season Code
	Heckled by 118
  ----------------------------------------------------------------------------*/

package org.usfirst.frc.team5414.robot;
import edu.wpi.first.wpilibj.CameraServer;
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
import org.usfirst.frc.team5414.robot.commands.SetAngle;
import org.usfirst.frc.team5414.robot.commands.ZeroEncoders;
import org.usfirst.frc.team5414.robot.commands.ZeroGyro;
import org.usfirst.frc.team5414.robot.subsystems.Arm;
import org.usfirst.frc.team5414.robot.subsystems.Climber;
import org.usfirst.frc.team5414.robot.subsystems.Drivetrain;
import org.usfirst.frc.team5414.robot.subsystems.IMU;
import org.usfirst.frc.team5414.robot.subsystems.Pincher;

public class Robot extends TimedRobot {
	
//	NetworkTable table = NetworkTable.getTable("limelight");
	public static Drivetrain drivetrain;
	public static Arm arm;
	public static Pincher pincher;
	public static Climber climber;
	public static OI oi;
	public static IMU gyro; 
	public static Compressor compressor;
	public static Preferences prefs;
	
	Command autonomousCommand;
	
	@Override
	public void robotInit() {
		drivetrain = new Drivetrain();
		arm = new Arm();
		pincher = new Pincher();
		climber = new Climber();
		oi = new OI();
		prefs = Preferences.getInstance();
		CameraServer.getInstance().startAutomaticCapture(0);
		if(!RobotMap.flatbot) compressor = new Compressor(0);
		if(RobotMap.flatbot) gyro = new IMU();
		
		if(!RobotMap.flatbot) compressor.start();
		if(RobotMap.flatbot) gyro.initialize();
		prefs.putDouble("Enc kP", RobotMap.kP);
		prefs.putDouble("Enc kI", RobotMap.kI);
		prefs.putDouble("Enc kD", RobotMap.kD);
		prefs.putDouble("Gyro kP", RobotMap.gykP);
		prefs.putDouble("Gyro kI", RobotMap.gykI);
		prefs.putDouble("Gyro kD", RobotMap.gykD);
		prefs.putInt("Desired Left Enc", 1440);
		prefs.putInt("Desired Right Enc", 1440);
		prefs.putDouble("Desired Angle", 0);
		SmartDashboard.putData("Test Drive Enc", new DriveEncDist(prefs.getInt("Desired Left Enc", 0), prefs.getInt("Desired Right Enc", 0)));
		SmartDashboard.putData("Zero Encoders", new ZeroEncoders());
		SmartDashboard.putData("Test Set Angle", new SetAngle(prefs.getDouble("Desired Angle", 0)));
		SmartDashboard.putData("Zero Gyro", new ZeroGyro());
	}

	@Override
	public void disabledInit() {
		
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		updateDashboard();
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
		updateDashboard();
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
		SmartDashboard.putData("Test Drive Enc", new DriveEncDist(prefs.getInt("Desired Left Enc", 0), prefs.getInt("Desired Right Enc", 0)));
		updateDashboard();
	}

	@Override
	public void testPeriodic() {
	}
	
	public void updateDashboard() {
		if(RobotMap.flatbot) SmartDashboard.putNumber("Current Yaw", gyro.getYaw());
		SmartDashboard.putNumber("Left Encoder", drivetrain.getEncoderL());
		SmartDashboard.putNumber("Right Encoder", drivetrain.getEncoderR());
	}
}
