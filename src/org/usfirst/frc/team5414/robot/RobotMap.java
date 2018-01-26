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
	
	//Ports for random parts
	public static int GyroPort = 0;
	
	//PWM for speed controllers
	public static int CANRightMotor1 = 12;
	public static int CANRightMotor2 = 16;
	public static int CANRightMotor3 = 13;
	public static int CANLeftMotor1 = 11;
	public static int CANLeftMotor2 = 14;
	public static int CANLeftMotor3 = 10;
	
	//Encoder DIO ports
	public static int DIOencoderFRa = 0;
	public static int DIOencoderFRb = 1;
	public static int DIOencoderFLa = 2;
	public static int DIOencoderFLb = 3;
	public static int DIOencoderBRa = 4;
	public static int DIOencoderBRb = 5;
	public static int DIOencoderBLa = 6;
	public static int DIOencoderBLb = 7;
	
	//Solenoid Ports
	public static int LShiftA = 0;
	public static int LShiftB = 1;
	public static int RShiftA = 2;
	public static int RShiftB = 3;
	
	//Wheel stuffs
	public static double wheelDiameterFeet = 6. / 12; //IN FEET
	public static double wheelDiameterMeters = wheelDiameterFeet * .3048; //IN FEET
	public static double EncoderPulsePerRev = 1440;
	public static double CircumferenceFeet = wheelDiameterFeet * Math.PI;
	public static double CircumferenceMeters = wheelDiameterMeters * Math.PI;
	public static double LengthPerTickFeet = CircumferenceFeet / EncoderPulsePerRev; 
	public static double LengthPerTickMeters = CircumferenceMeters / EncoderPulsePerRev; 
	public static double WheelBaseWidth = 0.6; //Meters
	
	//Motion Profiling/Encoder Following PID Loop
	public static double kP = .004;
	public static double kI = 0.00007;
	public static double kD = 0.007;
	
	//Gyro Turning PID Loop
	public static double gykP = 0.05;
	public static double gykI = 0;
	public static double gykD = 0.045;
}
