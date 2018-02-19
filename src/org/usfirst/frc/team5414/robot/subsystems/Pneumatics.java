package org.usfirst.frc.team5414.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Pneumatics extends Subsystem {

    DoubleSolenoid dsol1, dsol2, dsol3, dsol4;

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public Pneumatics() {
    	dsol1 = new DoubleSolenoid(0,7);
    	dsol2 = new DoubleSolenoid(1,6);
    	dsol3 = new DoubleSolenoid(2,5);
    	dsol4 = new DoubleSolenoid(3,4);
    }
    
    public void one() {
    	dsol1.set(DoubleSolenoid.Value.kReverse);
//    	dsol2.set(DoubleSolenoid.Value.kReverse);
//    	dsol3.set(DoubleSolenoid.Value.kReverse);
    	dsol4.set(DoubleSolenoid.Value.kForward);
    }
    
    public void two() {
    	dsol4.set(DoubleSolenoid.Value.kReverse);
    	dsol1.set(DoubleSolenoid.Value.kForward);
    }
    
    public void toggle() {
    	if(dsol4.get() == DoubleSolenoid.Value.kReverse) {
    		dsol4.set(DoubleSolenoid.Value.kForward);
    	}
    	else {
    		dsol4.set(DoubleSolenoid.Value.kReverse);
    	}  		
    }
}

