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
	public static boolean flatbot = false;
	public static boolean compbot = true;
	public static boolean hasLimelight = false;
	public static boolean hasGyro = false;
	public static boolean hasCompressor = false;
	public static boolean hasCam = false;
	public static boolean hasArm = true;
	public static boolean hasSpintake = true;
	
	//Ports for random parts
	public static int GyroPort = 0;
	
	//CAN ports for compbot speed controllers
	public static int CANRightSlave1 = 12; //Victor SPX
	public static int CANRightMaster = 16; //Talon SRX
	public static int CANRightSlave2 = 13; //Victor SPX
	public static int CANLeftSlave1 = 11; //Victor SPX
	public static int CANLeftMaster = 14; //Talon SRX
	public static int CANLeftSlave2 = 10; //Victor SPX
	
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
			public static double EncoderTicksPerRevFlat = 1440;
			public static double CircumferenceFeetFlat = wheelDiameterFeetFlat * Math.PI;
			public static double CircumferenceMetersFlat = wheelDiameterMetersFlat * Math.PI;
			public static double FeetPerTickFlat = CircumferenceFeetFlat / EncoderTicksPerRevFlat; 
			public static double MetersPerTickFlat = CircumferenceMetersFlat / EncoderTicksPerRevFlat; 
			//plybot
			public static double wheelDiameterFeet = 6. / 12; //IN FEET
			public static double wheelDiameterMeters = wheelDiameterFeet * .3048; //IN FEET
			public static double EncoderTicksPerRev = 128;
			public static double CircumferenceFeet = wheelDiameterFeet * Math.PI;
			public static double CircumferenceMeters = wheelDiameterMeters * Math.PI;
			public static double FeetPerTick = CircumferenceFeet / EncoderTicksPerRev;
			public static double MetersPerTick = CircumferenceMeters / EncoderTicksPerRev;
			public static double wheelBaseWidth = 5;
		
	//Motion Profiling/Encoder Following PID Loop
	public static double flatbotkP = .004;
	public static double flatbotkI = 0.00007;
	public static double flatbotkD = 0.007;
	public static double plybotLkF = .11;
	public static double plybotLkD = 0.0055;
	public static double plybotLkI = 0.00013;
	public static double plybotLkP = .0084;
	public static double plybotRkF = .11;
	public static double plybotRkD = 0.0034;
	public static double plybotRkI = 0.00011;
	public static double plybotRkP = 0.008;
	
	//Vision PID Loop
	public static double turnLimekD = 0.05;
	public static double turnLimekI = 0.00;
	public static double turnLimekP = 0.003;
	public static double forwardLimekD = 0.008;
	public static double forwardLimekI = 0.0;
	public static double forwardLimekP = 0.018;
	public static double forwardTurnLimekD = .3;
	public static double forwardTurnLimekI = 0.0;
	public static double forwardTurnLimekP = 0.007;
	
	//Arm PID Loop
	public static double armkP = 0.025;
	public static double armkI = 0.00009;
	public static double armkD = 0.2;
	public static double armThrowkP = 0.03;
	public static double armThrowkD = 0.15;

	//Motion Profile PID Loop (Jaci's Pathfinder)
	public static double MPkD = 0.0;
	public static double MPkI = 0.0;
	public static double MPkP = 0.0;

}
