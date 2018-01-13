package org.usfirst.frc.team5414.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Talon extends Subsystem {

	private TalonSRX talon;
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public Talon()
    {
//    	talon = new TalonSRX(0);
    }
    
    public void set(double percent)
    {
    	talon.set(ControlMode.PercentOutput, percent);
    }
}

