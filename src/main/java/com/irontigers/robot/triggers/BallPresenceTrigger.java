package com.irontigers.robot.triggers;

import com.irontigers.robot.sensors.IRRangefinder;

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
    private IRRangefinder presenceSensor;
    private MedianFilter sensorFilter;
    private int trigDist;

    public BallPresenceTrigger(int analogPort) {
        this(analogPort, 25);
    }

    public BallPresenceTrigger(int analogPort, int triggerDistance) {
        presenceSensor = new IRRangefinder(analogPort);
        sensorFilter = new MedianFilter(10);
        trigDist = triggerDistance;
    }

    public BallPresenceTrigger(IRRangefinder presenceSensor) {
        this(presenceSensor, 25);
    }

    public BallPresenceTrigger(IRRangefinder presenceSensor, int triggerDistance) {
        this.presenceSensor = presenceSensor;
        sensorFilter = new MedianFilter(10);
        trigDist = triggerDistance;
    }

    @Override
    public boolean get() {
        return sensorFilter.calculate(presenceSensor.getAverageDistance()) > trigDist;
    }

}