/*----------------------------------------------------------------------------
	Team 5414 Pearadox 2018 Build Season Code
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team5414.robot.commands.ArmSetAngle;
import org.usfirst.frc.team5414.robot.commands.ArmSetHover;
import org.usfirst.frc.team5414.robot.commands.ArmSetLow;
import org.usfirst.frc.team5414.robot.commands.ArmSetScale;
import org.usfirst.frc.team5414.robot.commands.ArmSetSwitch;
import org.usfirst.frc.team5414.robot.commands.ArmThrowbackHigh;
import org.usfirst.frc.team5414.robot.commands.AutoScaleLtoL;
import org.usfirst.frc.team5414.robot.commands.AutoScaleLtoR;
import org.usfirst.frc.team5414.robot.commands.AutoScaleRtoR;
import org.usfirst.frc.team5414.robot.commands.AutoSwitchLtoL;
import org.usfirst.frc.team5414.robot.commands.AutoSwitchMtoL;
import org.usfirst.frc.team5414.robot.commands.AutoSwitchMtoR;
import org.usfirst.frc.team5414.robot.commands.AutoSwitchRtoR;
import org.usfirst.frc.team5414.robot.commands.AutonomousDriveForward;
import org.usfirst.frc.team5414.robot.commands.AutonomousMidSwitch;
import org.usfirst.frc.team5414.robot.commands.AutonomousScalePriorityLeft;
import org.usfirst.frc.team5414.robot.commands.AutonomousScalePriorityRight;
import org.usfirst.frc.team5414.robot.commands.AutonomousSwitchMiddle;
import org.usfirst.frc.team5414.robot.commands.AutonomousSwitchPriorityLeft;
import org.usfirst.frc.team5414.robot.commands.AutonomousSwitchPriorityRight;
import org.usfirst.frc.team5414.robot.commands.DriveForward;
import org.usfirst.frc.team5414.robot.commands.FollowEncoder;
import org.usfirst.frc.team5414.robot.commands.TurnRight;
import org.usfirst.frc.team5414.robot.commands.VisionGoToCube;
import org.usfirst.frc.team5414.robot.commands.VisionTurnToCube;
import org.usfirst.frc.team5414.robot.commands.ZeroGyro;
import org.usfirst.frc.team5414.robot.subsystems.Arm;
import org.usfirst.frc.team5414.robot.subsystems.Drivetrain;
import org.usfirst.frc.team5414.robot.subsystems.IMU;
import org.usfirst.frc.team5414.robot.subsystems.Limelight;
import org.usfirst.frc.team5414.robot.subsystems.PDP;
import org.usfirst.frc.team5414.robot.subsystems.Spintake;

/*
 * Order of who to blame if the program doesn't work:
 * 0. electrical
 * 1. mechanical
 * 2. chairmans
 * 3. imagery
 * 4. scouting
 * 5. OpenMesh
 * 6. 118
 * 7. the butler
 * 8. Hayden Christensen's terrible acting in the prequels
 * 9. me? (nah)
 * 
 * Shoutout to mechanical for getting me 4 full minutes of the 
 * competition robot for autonomous testing during build season
 * 
 * Please love me Charles From 1477
 */

public class Robot extends TimedRobot {
	
	public static Drivetrain drivetrain;
	public static OI oi;
	public static IMU gyro; 
	public static Compressor compressor;
	public static Preferences prefs;
	public static Limelight limelight;
	public static Arm arm;
	public static Spintake spintake;
	public static I2C i2c = new I2C(Port.kOnboard, 4);
	public static PDP pdp;
	public static SendableChooser chooser;
	
	Command autonomousCommand;
	
	@Override
	public void robotInit() {
		chooser = new SendableChooser();
		drivetrain = new Drivetrain();
		prefs = Preferences.getInstance();
		limelight = new Limelight();
		if(RobotMap.hasArm)
		{
			arm = new Arm();	
		}
		if(RobotMap.hasSpintake) 
		{
			spintake = new Spintake();
//			pdp = new PDP();
		}
		oi = new OI();
		if(RobotMap.hasCompressor)
		{
			compressor = new Compressor(0);
			compressor.start();
		}
		if(RobotMap.hasGyro)
		{
			gyro = new IMU();
			gyro.initialize();
			
			SmartDashboard.putData("Zero Gyro", new ZeroGyro());
		}
		addPreferences();
		chooser.addDefault("Cross Baseline", new AutonomousDriveForward());
		chooser.addObject("Middle Switch", new AutonomousSwitchMiddle());
		chooser.addObject("Left Switch Priority", new AutonomousSwitchPriorityLeft());
		chooser.addObject("Right Switch Priority", new AutonomousSwitchPriorityRight());
		chooser.addObject("Left Scale Priority", new AutonomousScalePriorityLeft());
		chooser.addObject("Right Scale Priority", new AutonomousScalePriorityRight());
		SmartDashboard.putData("Autonomous Mode Chooser", chooser);
		SmartDashboard.putData("Turn Right", new TurnRight(90));
		SmartDashboard.putData("Drive Forward", new DriveForward(10));
		SmartDashboard.putData("Arm Switch", new ArmSetSwitch());
		SmartDashboard.putData("Arm Scale", new ArmSetScale());
		SmartDashboard.putData("Arm Low", new ArmSetLow());
		SmartDashboard.putData("Arm Hover", new ArmSetHover());
		SmartDashboard.putData("Arm Throwback", new ArmThrowbackHigh());
		if(RobotMap.hasLimelight)
		{
			SmartDashboard.putData("Vision Turn Cube", new VisionTurnToCube());
			SmartDashboard.putData("Vision Go To Cube", new VisionGoToCube());
		}
	}

	@Override
	public void disabledInit() {
		
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		if(RobotMap.compbot) updateDashboard();
		i2c.write(4, 0);
	}

	@Override
	public void autonomousInit() {
//		autonomousCommand = new AutonomousMidSwitch();
		autonomousCommand = (Command) chooser.getSelected();
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
		SmartDashboard.putData("Test Drive Encs", new FollowEncoder(prefs.getInt("Desired Left Enc", 0), prefs.getInt("Desired Right Enc", 0)));
		if(RobotMap.compbot) updateDashboard();
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
		if(RobotMap.hasGyro) SmartDashboard.putNumber("Current Yaw Raw", gyro.getYaw());
		if(RobotMap.hasGyro) SmartDashboard.putNumber("Current Yaw", gyro.getYaw()%360);
		if(RobotMap.hasArm)
		{
			SmartDashboard.putNumber("Arm Angle", arm.getAngle());
			SmartDashboard.putNumber("Arm Potentiometer", arm.getRaw());
		}
		SmartDashboard.putNumber("Left Encoder", drivetrain.getEncoderL());
		SmartDashboard.putNumber("Right Encoder", drivetrain.getEncoderR());
		SmartDashboard.putNumber("Left Velocity", drivetrain.getSpeedL());
		SmartDashboard.putNumber("Right Velocity", drivetrain.getSpeedR());
	}
	
	public void addPreferences() {
		prefs.putDouble("FlatEnc kP", RobotMap.flatbotkP);
		prefs.putDouble("FlatEnc kI", RobotMap.flatbotkI);
		prefs.putDouble("FlatEnc kD", RobotMap.flatbotkD);
		prefs.putDouble("PlyEnc L kF", RobotMap.plybotLkF);
		prefs.putDouble("PlyEnc L kP", RobotMap.plybotLkP);
		prefs.putDouble("PlyEnc L kI", RobotMap.plybotLkI);
		prefs.putDouble("PlyEnc L kD", RobotMap.plybotLkD);
		prefs.putDouble("PlyEnc R kF", RobotMap.plybotRkF);
		prefs.putDouble("PlyEnc R kP", RobotMap.plybotRkP);
		prefs.putDouble("PlyEnc R kI", RobotMap.plybotRkI);
		prefs.putDouble("PlyEnc R kD", RobotMap.plybotRkD);
		prefs.putDouble("Limelight kP", RobotMap.turnLimekP);
		prefs.putDouble("Limelight kD", RobotMap.turnLimekD);
		prefs.putDouble("Limelight Forward Turn kP", RobotMap.forwardTurnLimekP);
		prefs.putDouble("Limelight Forward Turn kI", RobotMap.forwardTurnLimekI);
		prefs.putDouble("Limelight Forward Turn kD", RobotMap.forwardTurnLimekD);
		prefs.putDouble("Limelight Forward kP", RobotMap.forwardLimekP);
		prefs.putDouble("Limelight Forward kD", RobotMap.forwardLimekD);
		prefs.putInt("Desired Left Enc", 20);
		prefs.putInt("Desired Right Enc", 20);
		prefs.putDouble("Arm kP", RobotMap.armkP);
		prefs.putDouble("Arm kI", RobotMap.armkI);
		prefs.putDouble("Arm kD", RobotMap.armkD);
		prefs.putDouble("Arm Throw kP", RobotMap.armThrowkP);
		prefs.putDouble("Arm Throw kD", RobotMap.armThrowkD);
		prefs.putDouble("Motion Profile kP", RobotMap.MPkP);
		prefs.putDouble("Motion Profile kI", RobotMap.MPkI);
		prefs.putDouble("Motion Profile kD", RobotMap.MPkD);
	}
}
