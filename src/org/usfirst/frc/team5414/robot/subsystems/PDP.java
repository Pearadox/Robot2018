package org.usfirst.frc.team5414.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class PDP extends Subsystem {

	PowerDistributionPanel pdp = new PowerDistributionPanel(1);
	
	public double getCurrent(int port) {
		return pdp.getCurrent(port);
	}
	
	public double getLeftSpintake() {
		return getCurrent(4);
 	}

	public double getRightSpintake() {
		return getCurrent(12);
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

