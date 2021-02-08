package com.irontigers.robot.sim;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class NavXSim extends SimDeviceSim {
    private SimDouble angle;
    private Rotation2d rot;

    public NavXSim() {
        super("navX-Sensor[0]");
        angle = getDouble("Yaw");
    }

    public void setAngle(double degrees) {
        angle.set(degrees);
    }
}
