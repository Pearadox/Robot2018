package org.usfirst.frc.team5414.robot;

import java.util.ArrayList;


public class TrajectoryGenerator {
	
	static double CRUISE_VELOCITY = 8;
	static double ACCELERATION = 3;

	public static Traj[] getTrajectory(double distance, double interval, double cruiseVelocity, double acceleration) //feet, seconds, ft/s, ft/s/s
    {
		CRUISE_VELOCITY = cruiseVelocity;
		ACCELERATION = acceleration;
        double accelTime = CRUISE_VELOCITY / ACCELERATION;
        double accelDistance = .5 * ACCELERATION * accelTime * accelTime * Math.copySign(1, distance);
        ArrayList<Traj> list = new ArrayList<>();
        
        if(Math.abs(accelDistance * 2) <= Math.abs(distance)) //trapezoidal
        {
            double rectangleDistance = (distance - accelDistance*2);
            double cruiseTime = Math.abs(rectangleDistance / CRUISE_VELOCITY);

           
            for(double time = 0 ;; time += interval)
            {
                if(time < accelTime)
                {
                    double s = ACCELERATION * time * Math.copySign(1, distance);
                    double a = ACCELERATION * Math.copySign(1, distance);
                    double d = .5 * s * time;
                    list.add(new Traj(s, d, a));
                }
                else if(time < cruiseTime + accelTime)
                {
                	double currentCruiseTime = time - accelTime;
                	double s = CRUISE_VELOCITY * Math.copySign(1, distance);
                	double a = 0;
                	double d = (accelDistance + currentCruiseTime * s);
                	list.add(new Traj(s, d, a));
                }
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
        else //triangular
        {
        	
            accelDistance = distance / 2. * Math.copySign(1, distance);
            accelTime = Math.sqrt(2 * Math.abs(accelDistance) / ACCELERATION);
            for(double time = 0 ;; time+=interval)
            {
	            if(time <= accelTime)
	            {
	                double s = ACCELERATION * time * Math.copySign(1, distance);
	                double a = ACCELERATION * Math.copySign(1, distance);
	                double d = .5 * s * time;
	                list.add(new Traj(s, d, a));
	            }
	            else if(time <= 2 * accelTime)
	            {
	                double peakVelocity = ACCELERATION * accelTime * Math.copySign(1, distance);
	                
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

