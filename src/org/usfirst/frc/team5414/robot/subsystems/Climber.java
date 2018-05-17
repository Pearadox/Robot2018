package org.usfirst.frc.team5414.robot.subsystems;

import org.usfirst.frc.team5414.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Climber extends Subsystem {
	
	DoubleSolenoid hook;
	VictorSPX motor;
	
	public Climber() {
		motor = new VictorSPX(RobotMap.CANClimberVictorSPX);
		hook = new DoubleSolenoid(1, 6);
	}
	
	public void toggleHook() {
		if(extended()) retract();
		else extend();
	}
	
	public void retract() {
		hook.set(DoubleSolenoid.Value.kReverse);
		stop();
	}
	
	public void extend() {
		hook.set(DoubleSolenoid.Value.kForward);
		stop();
	}
    
    public void setHold() {
    	motor.set(ControlMode.PercentOutput, .23);
    }
    
    public void setUp() {
    	motor.set(ControlMode.PercentOutput, 1.);
    }
    
    public void setDown() {
    	motor.set(ControlMode.PercentOutput, -.5);
    }
    
    public void stop() {
    	motor.set(ControlMode.PercentOutput, 0);
    }
    
    public boolean extended() {
    	return hook.get() == DoubleSolenoid.Value.kForward;
    }
    
	protected void initDefaultCommand() {
		
	}
}

