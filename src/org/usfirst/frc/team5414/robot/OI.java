/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5414.robot;

import java.io.IOException;

import org.usfirst.frc.team5414.robot.commands.ToggleClaw;
import org.usfirst.frc.team5414.robot.commands.ToggleLight;
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
		JoystickButton btn1 = new JoystickButton(stick, 1);
		JoystickButton btn2 = new JoystickButton(stick, 2);
		JoystickButton btn3 = new JoystickButton(stick, 3);
		JoystickButton btn4 = new JoystickButton(stick, 4);
		JoystickButton btn5 = new JoystickButton(stick, 5);
		JoystickButton btn6 = new JoystickButton(stick, 6);
		JoystickButton btn7 = new JoystickButton(stick, 7);
		JoystickButton btn8 = new JoystickButton(stick, 8);
		JoystickButton btn9 = new JoystickButton(stick, 9);
		JoystickButton btn10 = new JoystickButton(stick, 10);
		JoystickButton btn11 = new JoystickButton(stick, 11);
		JoystickButton btn12 = new JoystickButton(stick, 12);
		btn1.whenPressed(new ToggleClaw());
		btn2.whenPressed(new ToggleLight());
	}
	
	public Joystick getJoystick() {
		return stick;
		
	}
}
