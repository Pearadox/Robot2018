package org.usfirst.frc.team5414.robot.subsystems;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;
import org.usfirst.frc.team5414.robot.commands.ArmHold;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;


public class Arm extends Subsystem {

	TalonSRX talon;
	DoubleSolenoid solPincher;
	
	final double VHigh = 5;
	final double VLow = 0;
	final double angleLow = 0;
	final double angleHigh = 360;
	
	final double kP = 0;
	final double kI = 0;
	final double kD = 0;
	
	
	public Arm() {
		talon = new TalonSRX(RobotMap.CANArmTalon);
		talon.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 10);
		talon.setSensorPhase(true);
		talon.setInverted(false);
		talon.configNominalOutputForward(0, 10);
		talon.configNominalOutputReverse(0, 10);
		talon.configPeakOutputForward(1, 10);
		talon.configPeakOutputReverse(-1, 10);
		talon.config_kP(0, kP, 10);
		talon.config_kI(0, kI, 10);
		talon.config_kD(0, kD, 10);
		talon.configAllowableClosedloopError(0, 0, 10);
		
		solPincher = new DoubleSolenoid(2, 5);
	}
	
	public double getAngle() {
		return map(getAnalogIn(), VLow, VHigh, angleLow, angleHigh);
	}
	
	public void setAngle(double degrees)
	{
		talon.set(ControlMode.Position, map(degrees, angleLow, angleHigh, VLow, VHigh));
	}
	
	public double getAnalogIn() {
		return talon.getSensorCollection().getAnalogIn();
	}
	
	public double getError() { 
		return talon.getClosedLoopError(0);
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
	
	private double map(double x, double in_min, double in_max, double out_min, double out_max)
	{
	  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}
	
    public void initDefaultCommand() {
        setDefaultCommand(new ArmHold());
    }
}

