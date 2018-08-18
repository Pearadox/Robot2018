/*----------------------------------------------------------------------------
	Team 5414 Pearadox 2018 Post LSR Code
  ----------------------------------------------------------------------------*/

package org.usfirst.frc.team5414.robot;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import org.usfirst.frc.team5414.robot.commands.AutoScaleSwitchLtoL;
import org.usfirst.frc.team5414.robot.commands.AutoScaleSwitchRtoR;
import org.usfirst.frc.team5414.robot.commands.AutoScaleLtoR;
import org.usfirst.frc.team5414.robot.commands.AutoScaleRtoL;
import org.usfirst.frc.team5414.robot.commands.AutoScaleRtoR;
import org.usfirst.frc.team5414.robot.commands.AutoSwitchLtoL;
import org.usfirst.frc.team5414.robot.commands.AutoSwitchMtoL;
import org.usfirst.frc.team5414.robot.commands.AutoSwitchMtoR;
import org.usfirst.frc.team5414.robot.commands.AutoSwitchRtoR;
import org.usfirst.frc.team5414.robot.commands.AutonomousDriveForward;
import org.usfirst.frc.team5414.robot.commands.AutonomousScaleLeft;
import org.usfirst.frc.team5414.robot.commands.AutonomousSwitchMiddle;
import org.usfirst.frc.team5414.robot.commands.AutonomousSwitchOnlyLeft;
import org.usfirst.frc.team5414.robot.commands.AutonomousSwitchOnlyRight;
import org.usfirst.frc.team5414.robot.commands.AutonomousScalePriorityLeft;
import org.usfirst.frc.team5414.robot.commands.AutonomousScalePriorityRight;
import org.usfirst.frc.team5414.robot.commands.AutonomousScaleRight;
import org.usfirst.frc.team5414.robot.commands.AutonomousScaleSwitchLeft;
import org.usfirst.frc.team5414.robot.commands.AutonomousScaleSwitchRight;
import org.usfirst.frc.team5414.robot.commands.AutonomousSwitchMiddle;
import org.usfirst.frc.team5414.robot.commands.AutonomousSwitchPriorityLeft;
import org.usfirst.frc.team5414.robot.commands.AutonomousSwitchPriorityRight;
import org.usfirst.frc.team5414.robot.commands.DriveForward;
import org.usfirst.frc.team5414.robot.commands.DriveTimed;
import org.usfirst.frc.team5414.robot.commands.FollowEncoder;
import org.usfirst.frc.team5414.robot.commands.TurnRight;
import org.usfirst.frc.team5414.robot.commands.VisionGoToCube;
import org.usfirst.frc.team5414.robot.commands.VisionTurnToCube;
import org.usfirst.frc.team5414.robot.commands.ZeroGyro;
import org.usfirst.frc.team5414.robot.subsystems.Arm;
import org.usfirst.frc.team5414.robot.subsystems.Climber;
import org.usfirst.frc.team5414.robot.subsystems.Drivetrain;
import org.usfirst.frc.team5414.robot.subsystems.IMU;
import org.usfirst.frc.team5414.robot.subsystems.Limelight;
import org.usfirst.frc.team5414.robot.subsystems.PDP;
import org.usfirst.frc.team5414.robot.subsystems.Spintake;
// this is asdf

//this is master
/*
 * Order of who to blame if the program doesn't work:  
 * 0. electrical/Anyssa/angad
 * 1. mechanical
 * 2  OpenMesh
 * 3. chairmans
 * 4. scouting
 * 5. imagery
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
	public static Climber climber;
	
	
	public static boolean turnIsDone = true;
	DigitalInput Mid = new DigitalInput(0);
	DigitalInput Left = new DigitalInput(1);
	DigitalInput Scale = new DigitalInput(2);
	
	public static char switchSide = 'X';
	public static char scaleSide = 'X';
	
	
	Command autonomousCommand;
	
	@Override
	public void robotInit() {
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
		climber = new Climber();
		oi = new OI();
		addPreferences();
		SmartDashboard.putData("Turn Right", new TurnRight(90));
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
		limelight.lightOff();
	}

	@Override
	public void disabledInit() {
		limelight.lightOff();
		climber.stop();
	}

	@Override
	public void disabledPeriodic() {
		limelight.lightOff();
		Scheduler.getInstance().run();
		if(RobotMap.compbot) updateDashboard();
		i2c.write(4, 5); //changed from 0 to 5 for disabled arduino pattern
	}

	@Override
	public void autonomousInit() {
//		autonomousCommand = (Command) chooser.getSelected();
		limelight.lightOff();
		try {
			Thread.sleep(50);
		} catch (InterruptedException e1) {
			e1.printStackTrace();
		}
		String gameData = null;
		while(gameData == null)
			gameData = DriverStation.getInstance().getGameSpecificMessage();
		try {
	    	switchSide = gameData.charAt(0); // 'L' or 'R'
	    	scaleSide = gameData.charAt(1); // 'L' or 'R'
		} catch(Exception e) {System.out.println("Failed to retrieve FMS stuff");}
		SmartDashboard.putString("Game Data", gameData);
		boolean mid = Mid.get();
		boolean left = Left.get();
		boolean scale = Scale.get();
		if(mid) 
		{
			autonomousCommand = new AutonomousSwitchMiddle();
			SmartDashboard.putString("Auto Mode", "SwitchMiddle");
		}
		else if(switchSide == 'X') autonomousCommand = new AutonomousDriveForward();
		else if(left)
		{
			if(scale)				
			{
//				autonomousCommand = new AutonomousScaleSwitchLeft();
				autonomousCommand = new AutonomousScaleLeft();
				SmartDashboard.putString("Auto Mode", "ScaleLeft");
			}
			else 
			{
//				autonomousCommand = new AutonomousSwitchOnlyLeft();
				autonomousCommand = new AutonomousScalePriorityLeft();
				SmartDashboard.putString("Auto Mode", "SwitchLeft");
			}
		}
		else
		{
			if(scale)
			{
				autonomousCommand = new AutonomousScaleSwitchRight();
				SmartDashboard.putString("Auto Mode", "ScaleRight");
			}
			else 
			{
//				autonomousCommand = new AutonomousSwitchOnlyRight();
				autonomousCommand = new AutonomousScalePriorityRight();
				SmartDashboard.putString("Auto Mode", "SwitchRight");
			}
		}
		
//		autonomousCommand = new AutonomousSwitchMiddle();
//		autonomousCommand = new AutonomousScaleSwitchLeft();
//		autonomousCommand = new AutonomousScaleSwitchRight();
		 
//		autonomousCommand = new DriveForward(4);
		
		if (autonomousCommand != null) {
			autonomousCommand.start();
		}
	}


	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		updateDashboard();
		limelight.lightOff();
		i2c.write(4, 1);
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
		limelight.lightOff();
		drivetrain.setBrake();
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		limelight.lightOff();
		SmartDashboard.putData("Test Drive Encs", new FollowEncoder(prefs.getInt("Desired Left Enc", 0), prefs.getInt("Desired Right Enc", 0)));
		SmartDashboard.putData("DriveForwarddd", new DriveForward(prefs.getDouble("Forward Distance", 5)));
		if(RobotMap.compbot) updateDashboard();
		if(DriverStation.getInstance().getAlliance() == Alliance.Blue )
			i2c.write(4,  2);
		else
			i2c.write(4, 3);
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
		if(RobotMap.hasGyro)
		{
			SmartDashboard.putNumber("Current Yaw", gyro.getYaw()%360);
		}
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
		prefs.putDouble("Arm Throw High kP", RobotMap.armThrowHighkP);
		prefs.putDouble("Arm Throw High kD", RobotMap.armThrowHighkD);
		prefs.putDouble("Motion Profile kP", RobotMap.MPkP);
		prefs.putDouble("Motion Profile kI", RobotMap.MPkI);
		prefs.putDouble("Motion Profile kD", RobotMap.MPkD);
		prefs.putDouble("Forward Distance", 5);
	}
}
