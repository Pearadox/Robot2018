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
	
	//Ports for random parts
	public static int GyroPort = 0;
	
	//PWM for speed controllers
	public static int PWMRightMotor1 = 12;
	public static int PWMRightMotor2 = 16;
	public static int PWMRightMotor3 = 13;
	public static int PWMLeftMotor1 = 11;
	public static int PWMLeftMotor2 = 14;
	public static int PWMLeftMotor3 = 10;
	
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
	public static double wheelDiameterFeet = 4. / 12; //IN FEET
	public static double wheelDiameterMeters = wheelDiameterFeet * .3048; //IN FEET
	public static double EncoderTicks = 70;
	public static double CircumferenceFeet = wheelDiameterFeet * Math.PI;
	public static double CircumferenceMeters = wheelDiameterMeters * Math.PI;
	public static double LengthPerTickFeet = CircumferenceFeet / EncoderTicks; 
	public static double LengthPerTickMeters = CircumferenceMeters / EncoderTicks; 
	public static double WheelBaseWidth = 0.6; //Meters
	
	//Motion Profiling PID Loop
	public static double kP = 0;
	public static double kI = 0;
	public static double kD = 0;
}
