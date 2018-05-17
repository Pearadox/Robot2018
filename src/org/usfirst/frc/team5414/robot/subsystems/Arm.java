package org.usfirst.frc.team5414.robot.subsystems;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.RobotMap;
import org.usfirst.frc.team5414.robot.commands.ArmHold;
import org.usfirst.frc.team5414.robot.commands.ArmSetAngle;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Arm extends Subsystem {

	TalonSRX talon;
	DoubleSolenoid solPincher;
	AnalogInput potentiometer;
	
	//potentiometer parameters

	final static double VHigh = 1.64;
	final static double VLow = 3.985414;
	final static double angleLow = 45.1;
	final static double angleHigh = 180;
	final static double maxAngleStop = 180;
	final static double minAngleStop = 39; //49
	
	static boolean pinched;
	
	/*
	 * postive
	 * x` motor output will raise arm, negative will lower arm 
	 */
	public Arm() {
		talon = new TalonSRX(RobotMap.CANArmTalon);
		talon.setSensorPhase(true);
		talon.setInverted(false);
		talon.configNominalOutputForward(0, 10);
		talon.configNominalOutputReverse(0, 10);
		talon.configPeakOutputForward(1, 10);
		talon.configPeakOutputReverse(-1, 10);
		
		solPincher = new DoubleSolenoid(5, 2); //2, 5 on practice
		potentiometer = new AnalogInput(0);
		
		pinched = solPincher.get() == DoubleSolenoid.Value.kForward; 
	}
	
	public void setAngle(double angle)
	{
		Scheduler.getInstance().add(new ArmSetAngle(angle));
	}
	
	public double getAngle() {
		return map(getRaw(), VLow, VHigh, angleLow, angleHigh);
	}
	
	public double calculateHoldOutput(double angle)
	{
		SmartDashboard.putBoolean("Pinched", pinched);
		double amplitude = pinched ? .22 : 0.138;
		double equation = amplitude * Math.sin(angle*Math.PI/180);
		return equation;
	}
	
	public double getRaw() {
		return potentiometer.getVoltage();
	}
	
	public double getError() { 
		return talon.getClosedLoopError(0);
	}
	
	public void armUp() {
			double currentAngle = getAngle();
			set(calculateHoldOutput(currentAngle) + .6);
	}
	
	public void armDown() {
		try {
		double currentAngle = getAngle();
		set(calculateHoldOutput(currentAngle) - .3);
		} catch(Exception e) {set(-.4);}
	}
	
	public void armBreakaway() {
		set(-1.0);
	}
	
	
	
	public void set(double percentOutput)
	{
		if(percentOutput > 0 && getAngle() >= maxAngleStop) return;
		SmartDashboard.putNumber("Arm Output", percentOutput);
		talon.set(ControlMode.PercentOutput, percentOutput);
	}
	
	public void stop() {
		talon.set(ControlMode.PercentOutput, 0);
	}
	
	public void togglePincher() {
		if(pinched) openPincher();
		else closePincher();
	}
	
	public void closePincher() {
		pinched = true;
		solPincher.set(DoubleSolenoid.Value.kForward);
	}
	
	public void openPincher() {
		pinched = false;
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

