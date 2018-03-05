package org.usfirst.frc.team5414.robot.subsystems;

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
	DoubleSolenoid solLeft, solRight, solMiddle;
	boolean orienting = false;
	
	public Spintake()
	{
		left = new VictorSPX(22);
		right = new VictorSPX(21);
		solLeft = new DoubleSolenoid(0,7);
		solRight = new DoubleSolenoid(1, 6);
		solMiddle = new DoubleSolenoid(3, 4);
	}
	
	public void pushIn() {
		solLeft.set(DoubleSolenoid.Value.kReverse);
		solRight.set(DoubleSolenoid.Value.kReverse);
	}
	
	public void pushOut() {
		solLeft.set(DoubleSolenoid.Value.kForward);
		solRight.set(DoubleSolenoid.Value.kForward);
	}
	
	public void toggleMiddle() {
		if(solMiddle.get() == DoubleSolenoid.Value.kReverse) 
			solMiddle.set(DoubleSolenoid.Value.kForward);
    	else solMiddle.set(DoubleSolenoid.Value.kReverse);
	}
	
	public void intake()
	{
		left.set(ControlMode.PercentOutput, 1);
		right.set(ControlMode.PercentOutput, -1);
	}
	
	public void outtake()
	{
		left.set(ControlMode.PercentOutput, -1);
		right.set(ControlMode.PercentOutput, 1);
	}
	
	public void orient() {
		if(orienting)
		{
			stop();
			orienting = false;
			return;
		}
		left.set(ControlMode.PercentOutput, 1);
		right.set(ControlMode.PercentOutput, 0);
		orienting = true;
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
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

