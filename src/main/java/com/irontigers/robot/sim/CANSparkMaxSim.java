package com.irontigers.robot.sim;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class CANSparkMaxSim extends SimDeviceSim {
    private SimDouble postion;
    private SimDouble velocity;

    private double speed;

    public CANSparkMaxSim(int id) {
        super("SPARK MAX [" + id + "]");
        postion = getDouble("Position");
        velocity = getDouble("Velocity");
        speed = 0;
    }

    public void setPosition(double pos) {
        postion.set(pos);
    }

    public void setVelocity(double velocity) {
        this.velocity.set(velocity);
    }

}
