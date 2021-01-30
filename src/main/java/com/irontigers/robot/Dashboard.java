package com.irontigers.robot;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Dashboard {
    private static Dashboard instance;

    private ShuffleboardTab setupTab;
    private ShuffleboardTab driveTab;
    private ShuffleboardTab debugTab;

    private Dashboard() {}

    public static Dashboard getInstance() {
        if (instance == null) {
            instance = new Dashboard();
        }

        return instance;
    }
}