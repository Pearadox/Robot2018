package org.usfirst.frc.team5414.robot;

public class Traj {
	
	public double velocity = 0;
    public double distance = 0;
    public double acceleration = 0;

    public Traj(double spd, double dist, double accel)
    {
        velocity = spd;
        distance = dist;
        acceleration = accel;
    }

    @Override
    public String toString()
    {
        return String.format("%.2f", velocity) + "Ft/s | " + String.format("%.2f", distance)+ " Ft | " + acceleration + " Ft/s^2";
    }

}
