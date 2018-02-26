/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5414.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	//Fast switch between flatbot and 2018bot
	public static boolean flatbot = true;
	public static boolean compbot = false;
	public static boolean hasLimelight = false;
	public static boolean hasGyro = false;
	public static boolean hasCompressor = true;
	public static boolean hasCam = false;
	public static boolean hasArm = false;
	public static boolean hasSpintake = true;
	
	//Ports for random parts
	public static int GyroPort = 0;
	
	//CAN ports for compbot speed controllers
	public static int CANRightMotor1 = 12;
	public static int CANRightMotor2 = 16;
	public static int CANRightMotor3 = 13;
	public static int CANLeftMotor1 = 11;	
	public static int CANLeftMotor2 = 14;
	public static int CANLeftMotor3 = 10;
	
	//CAN ports for manipulators
	public static int CANArmTalon = 20;
	
	//Encoder DIO ports Flatbot
	public static int DIOencoderRaFlat = 0;
	public static int DIOencoderRbFlat = 1;
	public static int DIOencoderLaFlat = 2;
	public static int DIOencoderLbFlat = 3;
	public static int DIOencoderRaComp = 9;
	public static int DIOencoderRbComp = 8;
	public static int DIOencoderLaComp = 7;
	public static int DIOencoderLbComp = 6;
	
	
	//Solenoid Ports
	
	
	//Wheel stuffs
			//flatbot
			public static double wheelDiameterFeetFlat = 6. / 12; //IN FEET
			public static double wheelDiameterMetersFlat = wheelDiameterFeetFlat * .3048; //IN FEET
			public static double EncoderPulsePerRevFlat = 1440;
			public static double CircumferenceFeetFlat = wheelDiameterFeetFlat * Math.PI;
			public static double CircumferenceMetersFlat = wheelDiameterMetersFlat * Math.PI;
			public static double LengthPerTickFeetFlat = CircumferenceFeetFlat / EncoderPulsePerRevFlat; 
			public static double LengthPerTickMetersFlat = CircumferenceMetersFlat / EncoderPulsePerRevFlat; 
			//plybot
			public static double wheelDiameterFeetPly = 6. / 12; //IN FEET
			public static double wheelDiameterMetersPly = wheelDiameterFeetPly * .3048; //IN FEET
			public static double EncoderPulsePerRevPly = 256;
			public static double CircumferenceFeetPly = wheelDiameterFeetPly * Math.PI;
			public static double CircumferenceMetersPly = wheelDiameterMetersPly * Math.PI;
			public static double LengthPerTickFeetPly = CircumferenceFeetPly / EncoderPulsePerRevPly; 
			public static double LengthPerTickMetersPly = CircumferenceMetersPly / EncoderPulsePerRevPly; 
		
	//Motion Profiling/Encoder Following PID Loop
	public static double flatbotkP = .004;
	public static double flatbotkI = 0.00007;
	public static double flatbotkD = 0.007;
	public static double plybotLkF = .3;
	public static double plybotLkD = 0.0061;
	public static double plybotLkI = 0.00022;
	public static double plybotLkP = .013;
	public static double plybotRkF = .3;
	public static double plybotRkD = 0.0035;
	public static double plybotRkI = 0.000062;
	public static double plybotRkP = .01;
	
	//Vision PID Loop
	public static double turnLimekD = 0.01;
	public static double turnLimekI = 0.001;
	public static double turnLimekP = 0.002;
	public static double forwardLimekD = 0.03;
	public static double forwardLimekI = 0.01;
	public static double forwardLimekP = 0.085;
	public static double forwardTurnLimekD = 0.2;
	public static double forwardTurnLimekI = 0.00001;
	public static double forwardTurnLimekP = 0.2;
	
	//Gyro Turning PID Loop
//	public static double gykP = 0.01;
//	public static double gykI = 0.0001;
//	public static double gykD = 0;
//	public static double encodersLeftFullTurn = 815;
//	public static double encodersRightFullTurn = 438;
}
