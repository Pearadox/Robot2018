/*----------------------------------------------------------------------------
	Team 5414 Pearadox 2018 Build Season Code
	Heckled by 118
  ----------------------------------------------------------------------------*/

package org.usfirst.frc.team5414.robot;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team5414.robot.commands.AutonomousLeftToRightScale;
import org.usfirst.frc.team5414.robot.commands.DriveEncDist;
import org.usfirst.frc.team5414.robot.commands.SetAngle;
import org.usfirst.frc.team5414.robot.commands.TurnRight;
import org.usfirst.frc.team5414.robot.commands.VisionTurnToCube;
import org.usfirst.frc.team5414.robot.commands.ZeroEncoders;
import org.usfirst.frc.team5414.robot.commands.ZeroGyro;
import org.usfirst.frc.team5414.robot.subsystems.Drivetrain;
import org.usfirst.frc.team5414.robot.subsystems.IMU;
import org.usfirst.frc.team5414.robot.subsystems.Limelight;
import org.usfirst.frc.team5414.robot.subsystems.Pneumatics;

public class Robot extends TimedRobot {
	
//	NetworkTable table = NetworkTable.getTable("limelight");
	public static Drivetrain drivetrain;
	public static OI oi;
	public static IMU gyro; 
	public static Compressor compressor;
	public static Preferences prefs;
	public static Limelight limelight;
	public static Pneumatics pneumatics;
	public static I2C i2c = new I2C(Port.kOnboard, 4);
	
	Command autonomousCommand;
	
	@Override
	public void robotInit() {
		drivetrain = new Drivetrain();
		prefs = Preferences.getInstance();
		limelight = new Limelight();
		pneumatics = new Pneumatics();
		oi = new OI();
		if(RobotMap.hasCompressor)
		{
			compressor = new Compressor(0);
			compressor.start();
//			UsbCamera cam = new UsbCamera("cam0", "/dev/video0");
//			cam.setExposureAuto();
//			cam.setBrightness(50);
//			CameraServer.getInstance().startAutomaticCapture("cam0", "/dev/video0");
		}
		if(RobotMap.hasGyro)
		{
			gyro = new IMU();
			gyro.initialize();
			SmartDashboard.putData("Zero Gyro", new ZeroGyro());
		}
		addPreferences();
		SmartDashboard.putData("Zero Encoders", new ZeroEncoders());
		SmartDashboard.putData("Turn Right", new TurnRight(45));
		if(RobotMap.hasLimelight)
		{
			SmartDashboard.putData("Vision Turn Cube", new VisionTurnToCube());
		}

	}

	@Override
	public void disabledInit() {
		
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		if(!RobotMap.compbot) updateDashboard();
		i2c.write(4, 0);
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
		i2c.write(4, 2);
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
		if(!RobotMap.compbot) updateDashboard();
		i2c.write(4, 1);
	}

	@Override
	public void testPeriodic() {
	}
	
	public void updateDashboard() {
		if(RobotMap.hasLimelight)
		{
			SmartDashboard.putNumber("ty", Robot.limelight.getY());
			SmartDashboard.putNumber("ta", Robot.limelight.getArea());
		}
		if(RobotMap.hasGyro) SmartDashboard.putNumber("Current Yaw", gyro.getYaw());
		if(!RobotMap.flatbot)
		{
			SmartDashboard.putNumber("Left Encoder", drivetrain.getEncoderL());
			SmartDashboard.putNumber("Right Encoder", drivetrain.getEncoderR());
			SmartDashboard.putNumber("Left Encoder Feet", drivetrain.getEncoderLFeet());
			SmartDashboard.putNumber("Right Encoder Feet", drivetrain.getEncoderRFeet());
		}
	}
	
	public void addPreferences() {
		prefs.putDouble("FlatEnc kP", RobotMap.flatbotkP);
		prefs.putDouble("FlatEnc kI", RobotMap.flatbotkI);
		prefs.putDouble("FlatEnc kD", RobotMap.flatbotkD);
		prefs.putDouble("PlyEnc L kP", RobotMap.plybotLkP);
		prefs.putDouble("PlyEnc L kI", RobotMap.plybotLkI);
		prefs.putDouble("PlyEnc L kD", RobotMap.plybotLkD);
		prefs.putDouble("PlyEnc R kP", RobotMap.plybotRkP);
		prefs.putDouble("PlyEnc R kI", RobotMap.plybotRkI);
		prefs.putDouble("PlyEnc R kD", RobotMap.plybotRkD);
		prefs.putDouble("Limelight kP", RobotMap.turnLimekP);
		prefs.putDouble("Limelight kI", RobotMap.turnLimekI);
		prefs.putDouble("Limelight kD", RobotMap.turnLimekD);
		prefs.putDouble("Limelight Forward kP", RobotMap.forwardLimekP);
		prefs.putDouble("Limelight Forward kD", RobotMap.forwardLimekD);
		prefs.putInt("Desired Left Enc", 300);
		prefs.putInt("Desired Right Enc", 300);
		prefs.putDouble("Desired Angle", 0);
		prefs.putDouble("Vision Error", limelight.getX());
		
	}
}
//