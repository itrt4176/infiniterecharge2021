// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.irontigers.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

/** Add your docs here. */
public class IRRangefinder extends AnalogInput {
    public IRRangefinder(final int channel) {
        super(channel);
        SendableRegistry.addLW(this, "IRRangefinder", channel);
    }

    private double scaleToDistance(double voltage) {
        return 29.988 * Math.pow(voltage, -1.173);
    }

    public double getDistance() {
        return scaleToDistance(getVoltage());
    }

    public double getAverageDistance() {
        return scaleToDistance(getAverageVoltage());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Analog Input");
        builder.addDoubleProperty("Value", this::getAverageDistance, null);
    }
}
