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

	final static double VHigh = 1.448;
	final static double VLow = 4.12;
	final static double angleLow = 34.8;
	final static double angleHigh = 180;
	final static double maxAngleStop = 165;
	final static double minAngleStop = 39; //49
	
	static boolean pinched;
	
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
		double amplitude = pinched ? 0.15 : 0.143; //if there's a cube, the magnitude must be higher due to more TORQUE
		double equation = amplitude * Math.sin(angle*Math.PI/180);
		SmartDashboard.putNumber("calculate", equation);
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
		if(currentAngle <= minAngleStop)
			set(calculateHoldOutput(currentAngle));
		set(calculateHoldOutput(currentAngle) - .6);
		} catch(Exception e) {set(-.4);}
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
		System.out.println(pinched);
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

