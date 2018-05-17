package org.usfirst.frc.team5414.robot.subsystems;

import org.usfirst.frc.team5414.robot.Robot;
import org.usfirst.frc.team5414.robot.commands.ArmHold;
import org.usfirst.frc.team5414.robot.commands.SpintakePOVControl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Spintake extends Subsystem {

	VictorSPX left, right;
	DoubleSolenoid sol;
	static boolean orienting = false;
	
	public Spintake()
	{
		left = new VictorSPX(22);
		right = new VictorSPX(21);
		left.setInverted(true); //false on practice
		right.setInverted(false); //true on practice
		sol = new DoubleSolenoid(4,3);
	}
	
	public void pushIn() {
		SmartDashboard.putBoolean("Pushed In", true);
		sol.set(DoubleSolenoid.Value.kReverse);
	}
	
	public void pushOut() {
		SmartDashboard.putBoolean("Pushed In", false);
		sol.set(DoubleSolenoid.Value.kForward);
	}
	
	public void pushOff() {
		sol.set(DoubleSolenoid.Value.kOff);
	}
	
	public void intake()
	{
		setCoast();
		if(!orienting)
		{
			left.set(ControlMode.PercentOutput, .7);
			right.set(ControlMode.PercentOutput, .7);
		}
	}
	
	public void outtake()
	{
		setCoast();
		left.set(ControlMode.PercentOutput, -.7);
		right.set(ControlMode.PercentOutput, -.7);
	}
	
	public void orientLeft() {
		setCoast();
		left.set(ControlMode.PercentOutput, .9);
		right.set(ControlMode.PercentOutput, 0);
	}
	
	public void orientRight() {
		setCoast();
		left.set(ControlMode.PercentOutput, 0);
		right.set(ControlMode.PercentOutput, .9);
	}
	
	public void setOrienting(boolean b)
	{
		orienting = b;
	}
	
	public void stop() 
	{
		setBrake();
		left.set(ControlMode.PercentOutput, 0);
		right.set(ControlMode.PercentOutput, 0);
	}
	
	public void stopNoBrake() {
		left.set(ControlMode.PercentOutput, 0);
		right.set(ControlMode.PercentOutput, 0);
	}
	
	public void setLeft(double percentOutput)
	{
		left.set(ControlMode.PercentOutput, percentOutput);
	}
	
	public void setRight(double percentOutput)
	{
		right.set(ControlMode.PercentOutput, percentOutput);
	}
	
	public void setCoast()
	{
		left.setNeutralMode(NeutralMode.Coast);
		right.setNeutralMode(NeutralMode.Coast);
	}
	
	public void setBrake()
	{
		left.setNeutralMode(NeutralMode.Brake);
		right.setNeutralMode(NeutralMode.Brake);
	}

    public void initDefaultCommand() {
    	setDefaultCommand(new SpintakePOVControl());
    }
}

