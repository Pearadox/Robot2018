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

/**
 *
 */
public class Spintake extends Subsystem {

	VictorSPX left, right;
	DoubleSolenoid sols, solMiddle;
	static boolean orienting = false;
	
	public Spintake()
	{
		left = new VictorSPX(22);
		right = new VictorSPX(21);
		left.setInverted(false);
		right.setInverted(true);
		sols = new DoubleSolenoid(3,4);
	}
	
	public void pushIn() {
		sols.set(DoubleSolenoid.Value.kReverse);
	}
	
	public void pushOut() {
		sols.set(DoubleSolenoid.Value.kForward);
	}
	
	public void pushOff() {
		sols.set(DoubleSolenoid.Value.kOff);
	}
	
	public void toggleMiddle() {
		if(solMiddle.get() == DoubleSolenoid.Value.kReverse) 
			solMiddle.set(DoubleSolenoid.Value.kForward);
    	else solMiddle.set(DoubleSolenoid.Value.kReverse);
	}
	
	public void intake()
	{
		setCoast();
		if(!orienting)
		{
			left.set(ControlMode.PercentOutput, .95  );
			right.set(ControlMode.PercentOutput, .95);
		}
	}
	
	public void outtake()
	{
		setCoast();
		left.set(ControlMode.PercentOutput, -.7);
		right.set(ControlMode.PercentOutput, -.7);
	}
	
	public void orientLeft() {
//		if(orienting)
//		{
//			stop();
//			orienting = false;
//			return;
//		}
		setCoast();
		left.set(ControlMode.PercentOutput, .7);
		right.set(ControlMode.PercentOutput, 0);
//		orienting = true;
	}
	
	public void orientRight() {
		setCoast();
		left.set(ControlMode.PercentOutput, -.5);
		right.set(ControlMode.PercentOutput, .5);
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

