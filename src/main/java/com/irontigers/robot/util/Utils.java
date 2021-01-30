package com.irontigers.robot.util;

public final class Utils {

    /**
     * Utility class, so constructor is private.
     */
    private Utils() {
    throw new UnsupportedOperationException("This is a utility class!");
    }

    public static double pulsesToWheelRot(double pulses, double ppr, double wheelCircumference) {
        return pulses * 1/ppr * wheelCircumference;
    }

    public static double pulsesToWheelRot(double pulses, double ppr, double wheelCircumference, double motorTeeth, double wheelTeeth) {
        return pulses * (motorTeeth / wheelTeeth) * (1 / ppr) * wheelCircumference;
    }

    public static double wheelRotToPulses(double rotations, double ppr, double wheelCircumference) {
        return rotations * ppr * (1 / wheelCircumference);
    }

    public static double wheelRotToPulses(double rotations, double ppr, double wheelCircumference, double motorTeeth, double wheelTeeth) {
        return rotations * ppr * (wheelTeeth / motorTeeth) * (1 / wheelCircumference);
    }

    public static double deadZone(double controllerOutput) {
        return (controllerOutput > 0.05 || controllerOutput < -0.05) ? controllerOutput : 0.0;
    } 
}