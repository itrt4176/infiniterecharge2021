package com.irontigers.robot.sim;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class CANSparkMaxSim extends SimDeviceSim {
    private SimDouble postion;
    private SimDouble velocity;

    public CANSparkMaxSim(int id) {
        super("SPARK MAX [" + id + "]");
        postion = getDouble("Position");
        velocity = getDouble("Velocity");
    }

    public void setPosition(double pos) {
        postion.set(pos);
    }

    public void setVelocity(double velocity) {
        this.velocity.set(velocity);
    }

}
