package com.irontigers.robot.triggers;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * Function that extends {@link Trigger} using an analog distance sensor.
 * 
 * <p>When a ball is close enough to the sensor, it will be "triggered".
 */
public class BallPresenceTrigger extends Trigger {
    
    /** Analog distance sensors behave identically  */
    private AnalogInput presenceSensor;
    private MedianFilter sensorFilter;

    public BallPresenceTrigger(int analogPort) {
        presenceSensor = new AnalogInput(analogPort);
        sensorFilter = new MedianFilter(10);
    }

    public BallPresenceTrigger(AnalogInput presenceSensor) {
        this.presenceSensor = presenceSensor;
        sensorFilter = new MedianFilter(10);
    }

    @Override
    public boolean get() {
        return sensorFilter.calculate(presenceSensor.getAverageValue()) > 1500;
    }

}