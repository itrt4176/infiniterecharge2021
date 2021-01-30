package com.irontigers.robot;

import edu.wpi.first.wpilibj.Solenoid;

public class LedController {
    private Solenoid leds;

    public LedController(int pcmPort) {
        leds = new Solenoid(pcmPort);
    }

    public void enableLeds() {
        leds.set(true);
    }

    public void disableLeds() {
        leds.set(false);
    }

    public boolean ledsOn() {
        return leds.get();
    }
}