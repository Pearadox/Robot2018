/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5414.robot;

import java.io.IOException;

import org.usfirst.frc.team5414.robot.commands.ShiftDown;
import org.usfirst.frc.team5414.robot.commands.ShiftNone;
import org.usfirst.frc.team5414.robot.commands.ShiftUp;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	
	public static Joystick stick = new Joystick(0);

	public OI() 
	{
		JoystickButton record = new JoystickButton(stick, 11);
		JoystickButton up = new JoystickButton(stick, 3);
		JoystickButton down = new JoystickButton(stick, 4);
		up.whenPressed(new ShiftUp());
		down.whenPressed(new ShiftDown());
	}
	
	public Joystick getJoystick() {
		return stick;
		
	}
}
