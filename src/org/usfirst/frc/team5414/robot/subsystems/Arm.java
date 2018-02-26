package org.usfirst.frc.team5414.robot.subsystems;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;


public class Arm extends Subsystem {

	TalonSRX talon;
	DoubleSolenoid solPincher;
	
	public Arm() {
		talon = new TalonSRX(RobotMap.CANArmTalon);
		solPincher = new DoubleSolenoid(2, 5);
	}
	
	public void armUp() {
		talon.set(ControlMode.PercentOutput, -.7);
	}
	
	public void armDown() {
		talon.set(ControlMode.PercentOutput, .4);
	}
	
	public void stop() {
		talon.set(ControlMode.PercentOutput, 0);
	}
	
	public void togglePincher() {
		if(solPincher.get() == DoubleSolenoid.Value.kReverse) 
			solPincher.set(DoubleSolenoid.Value.kForward);
    	else solPincher.set(DoubleSolenoid.Value.kReverse);
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

