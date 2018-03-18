package org.usfirst.frc.team5414.robot;

import java.util.ArrayList;


public class TrajectoryGenerator {
	
	//METERS
	static double CRUISE_VELOCITY = 2.5;
	static double ACCELERATION = 2;

	public static Traj[] getTrajectory(double distance, double interval, double cruiseVelocity, double acceleration) //meters, seconds, m/s, m/s/s
    {
		CRUISE_VELOCITY = cruiseVelocity;
		ACCELERATION = acceleration;

		//Calculate trapezoidal distance to determine whether to use trapezoidal or triangular motion profile
        double accelTime = CRUISE_VELOCITY / ACCELERATION;
        double accelDistance = .5 * ACCELERATION * accelTime * accelTime * Math.copySign(1, distance);
        
        ArrayList<Traj> list = new ArrayList<>();        
        if(Math.abs(accelDistance * 2) <= Math.abs(distance)) //trapezoidal
        {
            double rectangleDistance = (distance - accelDistance*2);
            double cruiseTime = Math.abs(rectangleDistance / CRUISE_VELOCITY);
            
            //create trajectory for every single point in time
            for(double time = 0 ;; time += interval)
            {
            	//section 1: accelerating to cruise velocity
                if(time < accelTime)
                {
                    double s = ACCELERATION * time * Math.copySign(1, distance);
                    double a = ACCELERATION * Math.copySign(1, distance);
                    double d = .5 * s * time;
                    list.add(new Traj(s, d, a));
                }
                //section 2: cruising along at CRUISE_VELOCITY
                else if(time < cruiseTime + accelTime)
                {
                	double currentCruiseTime = time - accelTime;
                	double s = CRUISE_VELOCITY * Math.copySign(1, distance);
                	double a = 0;
                	double d = (accelDistance + currentCruiseTime * s);
                	list.add(new Traj(s, d, a));
                }
                //section 3: decelerating from cruise velocity to 0
                else if(time < cruiseTime + 2*accelTime)
                {
                    double timeAfterDecelerationStarted = time - cruiseTime - accelTime;
                    double decelVelocity = (CRUISE_VELOCITY - ACCELERATION * timeAfterDecelerationStarted) * Math.copySign(1, distance);
                    if(distance > 0 && decelVelocity < 0) decelVelocity = 0;
                    else if(distance < 0 && decelVelocity > 0) decelVelocity = 0;
                    double s = decelVelocity;
                    double a = -ACCELERATION * Math.copySign(1, distance);
                    if(s == 0) a = 0;
                    double d = accelDistance + rectangleDistance + timeAfterDecelerationStarted * CRUISE_VELOCITY / 2 * Math.copySign(1, distance);
                    list.add(new Traj(s, d, a));
                }
                else break;
            }
            
        }
        else //triangular: used if trapezoidal will not fit into the desired distance
        {
            accelDistance = distance / 2. * Math.copySign(1, distance);
            accelTime = Math.sqrt(2 * Math.abs(accelDistance) / ACCELERATION);
            
            //peak velocity is reached when robot is halfway to the desired distance
            double peakVelocity = ACCELERATION * accelTime * Math.copySign(1, distance); 
            for(double time = 0 ;; time+=interval)
            {
            	//section 1: accelerate to peak velocity
	            if(time <= accelTime)
	            {
	                double s = ACCELERATION * time * Math.copySign(1, distance);
	                double a = ACCELERATION * Math.copySign(1, distance);
	                double d = .5 * s * time;
	                list.add(new Traj(s, d, a));
	            }
	            //section 2: decelerate from peak velocity to 0
	            else if(time <= 2 * accelTime)
	            {
	                double decelVelocity = peakVelocity - ACCELERATION * (time - accelTime) * Math.copySign(1, distance);
	                if(distance > 0 && decelVelocity < 0) decelVelocity = 0;
                    else if(distance < 0 && decelVelocity > 0) decelVelocity = 0;
	                double s = decelVelocity;
	                double a = -ACCELERATION * Math.copySign(1, distance);
	                if(decelVelocity == 0) a = 0;
	                double d = distance - (accelTime * 2 - time) * s / 2;
	                list.add(new Traj(s, d, a));
	            }
	            else break;
        	}
        }
        return list.toArray(new Traj[list.size()]);
    }
}

