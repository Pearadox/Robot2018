/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5414.robot;

import java.io.IOException;

import org.usfirst.frc.team5414.robot.commands.ArmDownManual;
import org.usfirst.frc.team5414.robot.commands.ArmUpManual;
import org.usfirst.frc.team5414.robot.commands.SpintakeIntake;
import org.usfirst.frc.team5414.robot.commands.Outtake;
import org.usfirst.frc.team5414.robot.commands.SpintakeOrient;
import org.usfirst.frc.team5414.robot.commands.SpintakePushIn;
import org.usfirst.frc.team5414.robot.commands.SpintakePushOut;
import org.usfirst.frc.team5414.robot.commands.SpintakeToggleMiddle;
import org.usfirst.frc.team5414.robot.commands.ToggleClaw;
import org.usfirst.frc.team5414.robot.commands.ToggleLight;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
Commands/Functions we will probably need:
	arm manual up
	arm manual down
	set arm to down position
	set arm to switch position
	set arm to scale position
	toggle pincher on arm
	spintake orient
	spintake intake
	spintake outtake
	spintake push in (pneumatics)
	spintake push out (pneumatics)
	vision turn/go to cube (driver assitance, might use, might not)
	hold to go straight (driver assistance, might use, probably not)
	throw scale (maybe one for throwing upwards from the front and one for throwing from the back?)
	throw switch
Current Total: 15/16
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
		
		btn2.whenPressed(new ToggleLight());
		
		
		btn1.whenPressed(new ToggleClaw()); //temporary
		if(RobotMap.hasArm)
		{
//			btn1.whenPressed(new ToggleClaw());
			btn7.whileHeld(new ArmDownManual());
			btn8.whileHeld(new ArmUpManual());
		}
		if(RobotMap.hasSpintake)
		{
			btn3.whenPressed(new SpintakeToggleMiddle());
			btn4.whileHeld(new SpintakeOrient());
			btn5.whenPressed(new SpintakePushIn());
			btn6.whenPressed(new SpintakePushOut());
			btn9.whileHeld(new SpintakeIntake());
			btn10.whileHeld(new Outtake());
		}
	}
	
	public Joystick getJoystick() {
		return stick;
		
	}
}
