package org.usfirst.frc.team5414.robot.subsystems;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;
import org.usfirst.frc.team5414.robot.commands.ArmHold;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;


public class Arm extends Subsystem {

	TalonSRX talon;
	DoubleSolenoid solPincher;
	AnalogInput potentiometer;
	
	//potentiometer parameters
	final static double VHigh = 1.407;
	final static double VLow = 4.288;
	final static double angleLow = 38.7;
	final static double angleHigh = 190.7;
	final static double maxAngleStop = 155;
	
	final double kP = 0;
	final double kI = 0;
	final double kD = 0;
	
	/*
	 * postive motor output will raise arm, negative will lower arm
	 */
	public Arm() {
		talon = new TalonSRX(RobotMap.CANArmTalon);
//		talon.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 10);
		talon.setSensorPhase(true);
		talon.setInverted(false);
		talon.configNominalOutputForward(0, 10);
		talon.configNominalOutputReverse(0, 10);
		talon.configPeakOutputForward(1, 10);
		talon.configPeakOutputReverse(-1, 10);
//		talon.config_kP(0, kP, 10);
//		talon.config_kI(0, kI, 10);
//		talon.config_kD(0, kD, 10);
//		talon.configAllowableClosedloopError(0, 0, 10);
		
		solPincher = new DoubleSolenoid(2, 5);
		potentiometer = new AnalogInput(0);
	}
	
	public double getAngle() {
		return map(getAnalogIn(), VLow, VHigh, angleLow, angleHigh);
	}
	
	public double calculateHoldOutput(double angle)
	{
//		if(angle >= 165) return 0;
		return 0.155 * Math.sin(0.0175 * angle);
	}
	
	private double getAnalogIn() {
		return potentiometer.getVoltage();
	}
	
	public double getError() { 
		return talon.getClosedLoopError(0);
	}
	
	public void armUp() {
		double currentAngle = getAngle();
		if(currentAngle >= maxAngleStop)
		{
			set(calculateHoldOutput(currentAngle));
		}
		else set(.7);
	}
	
	public void armDown() {
		set(-.4);
	}
	
	public void set(double percentOutput)
	{
		if(percentOutput > 0 && getAngle() >= maxAngleStop) return;
		talon.set(ControlMode.PercentOutput, percentOutput);
	}
	
	public void stop() {
		talon.set(ControlMode.PercentOutput, 0);
	}
	
	public void togglePincher() {
		if(solPincher.get() == DoubleSolenoid.Value.kReverse) 
			solPincher.set(DoubleSolenoid.Value.kForward);
    	else solPincher.set(DoubleSolenoid.Value.kReverse);
	}
	
	public void closePincher() {
		solPincher.set(DoubleSolenoid.Value.kForward);
	}
	
	public void openPincher() {
		solPincher.set(DoubleSolenoid.Value.kReverse);
	}
	
	private double map(double x, double in_min, double in_max, double out_min, double out_max)
	{
	  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}
	
    public void initDefaultCommand() {
        setDefaultCommand(new ArmHold());
    }
}

