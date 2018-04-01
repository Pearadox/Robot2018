/*----------------------------------------------------------------------------*/

/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5414.robot;

import java.io.IOException;

import org.usfirst.frc.team5414.robot.commands.ArmDownManual;
import org.usfirst.frc.team5414.robot.commands.ArmPincherClose;
import org.usfirst.frc.team5414.robot.commands.ArmPincherOpen;
import org.usfirst.frc.team5414.robot.commands.ArmUpManual;
import org.usfirst.frc.team5414.robot.commands.SpintakeIntake;
import org.usfirst.frc.team5414.robot.commands.SpintakeOuttake;
import org.usfirst.frc.team5414.robot.commands.SpintakeOrientLeft;
import org.usfirst.frc.team5414.robot.commands.SpintakeOrientRight;
import org.usfirst.frc.team5414.robot.commands.SpintakePushIn;
import org.usfirst.frc.team5414.robot.commands.SpintakePushOff;
import org.usfirst.frc.team5414.robot.commands.SpintakePushOut;
import org.usfirst.frc.team5414.robot.commands.SpintakeToggleMiddle;
import org.usfirst.frc.team5414.robot.commands.ArmPincherToggle;
import org.usfirst.frc.team5414.robot.commands.ArmSetLow;
import org.usfirst.frc.team5414.robot.commands.ArmSetScale;
import org.usfirst.frc.team5414.robot.commands.ArmSetSwitch;
import org.usfirst.frc.team5414.robot.commands.ArmThrowbackHigh;
import org.usfirst.frc.team5414.robot.commands.ArmThrowbackHighGroup;
import org.usfirst.frc.team5414.robot.commands.ArmThrowbackLow;
import org.usfirst.frc.team5414.robot.commands.ArmThrowbackLowGroup;
import org.usfirst.frc.team5414.robot.commands.ToggleLight;
import org.usfirst.frc.team5414.robot.commands.VisionGoToCube;
import org.usfirst.frc.team5414.robot.commands.VisionTurnToCube;

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
	public static Joystick operator = new Joystick(1);

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
		
//		/* Bharath's mapping
		btn1.whenPressed(new ArmPincherToggle());
		btn2.whileHeld(new VisionTurnToCube());
		btn3.whenPressed(new ArmThrowbackLowGroup());
		btn4.whenPressed(new ArmSetLow());
		btn5.whenPressed(new ArmSetSwitch());
		btn6.whenPressed(new ArmSetScale());
		btn7.whileHeld(new ArmDownManual());
		btn8.whileHeld(new ArmUpManual());
		
		btn9.whileHeld(new SpintakeIntake());
		btn10.whileHeld(new SpintakeOuttake());
		btn11.whenPressed(new SpintakePushIn());
		btn12.whenPressed(new SpintakePushOut());
//		*/
		
		//------OPERATOR-------------------------
		JoystickButton btnOp1 = new JoystickButton(operator, 1);
		JoystickButton btnOp2 = new JoystickButton(operator, 2);
		JoystickButton btnOp3 = new JoystickButton(operator, 3);
		JoystickButton btnOp4 = new JoystickButton(operator, 4);
		JoystickButton btnOp5 =	new JoystickButton(operator, 5);
		JoystickButton btnOp6 = new JoystickButton(operator, 6);
		JoystickButton btnOp7 = new JoystickButton(operator, 7);
		JoystickButton btnOp8 = new JoystickButton(operator, 8);
		JoystickButton btnOp9 = new JoystickButton(operator, 9);
		JoystickButton btnOp10 = new JoystickButton(operator, 10);
		JoystickButton btnOp13 = new JoystickButton(operator, 13);
		
		
		btnOp1.whenPressed(new ArmThrowbackHigh());
		btnOp2.whileHeld(new SpintakeIntake());
		btnOp3.whenPressed(new ArmSetLow());
		btnOp4.whenPressed(new ArmSetSwitch());
		btnOp5.whenPressed(new SpintakePushOut());
		btnOp6.whileHeld(new VisionTurnToCube());
		btnOp7.whenPressed(new SpintakePushIn());
		btnOp8.whileHeld(new VisionGoToCube());
//		btnOp9.whileHeld(new SpintakeOrientLeft());
//		btnOp10.whileHeld(new SpintakeOrientRight());
//		btnOp13.whileHeld(new SpintakeOrientRight());
	}
	
	public Joystick getJoystick() {
		return stick;
		
	}
	
	public Joystick getOpJoystick() {
		return operator;
	}
}
