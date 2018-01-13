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
	public static int PWMRightFrontMotor = 2;
	public static int PWMRightBackMotor = 3;
	public static int PWMLeftFrontMotor = 0;
	public static int PWMLeftBackMotor = 1;
	
	//Encoder DIO ports
	public static int DIOencoderFRa = 8;
	public static int DIOencoderFRb = 9;
	public static int DIOencoderFLa = 4;
	public static int DIOencoderFLb = 5;
	public static int DIOencoderBRa = 6;
	public static int DIOencoderBRb = 7;
	public static int DIOencoderBLa = 2;
	public static int DIOencoderBLb = 3;
	
	//Wheel stuffs
	public static double wheelDiameter = 4. / 12; //IN FEET
	public static double EncoderTicks = 70;
	public static double Circumference = wheelDiameter * Math.PI;
	public static double LengthPerTick = Circumference / EncoderTicks; // this is only roughly true for traction wheels
	public static double WheelBaseWidth = 0.6;
}
