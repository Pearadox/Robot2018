package org.usfirst.frc.team5414.robot.subsystems;

import org.usfirst.frc.team5414.robot.commands.ArmHold;
import org.usfirst.frc.team5414.robot.commands.SpintakePOVControl;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
	boolean orienting = false;
	
	public Spintake()
	{
		left = new VictorSPX(22);
		right = new VictorSPX(21);
		left.setInverted(true);
		right.setInverted(true);
		sols = new DoubleSolenoid(3,4);
//		solRight = new DoubleSolenoid(1, 6);
//		solMiddle = new DoubleSolenoid(3, 4);
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
		System.out.println("in");
		left.set(ControlMode.PercentOutput, -.8  );
		right.set(ControlMode.PercentOutput, .8);
	}
	
	public void outtake()
	{
		System.out.println("out");
		left.set(ControlMode.PercentOutput, .7);
		right.set(ControlMode.PercentOutput, -.7);
	}
	
	public void orientLeft() {
//		if(orienting)
//		{
//			stop();
//			orienting = false;
//			return;
//		}
		left.set(ControlMode.PercentOutput, -.7);
		right.set(ControlMode.PercentOutput, 0);
//		orienting = true;
	}
	
	public void orientRight() {
		left.set(ControlMode.PercentOutput, 0);
		right.set(ControlMode.PercentOutput, .7);
	}
	
	public void stop() 
	{
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

    public void initDefaultCommand() {
    	setDefaultCommand(new SpintakePOVControl());
    }
}

