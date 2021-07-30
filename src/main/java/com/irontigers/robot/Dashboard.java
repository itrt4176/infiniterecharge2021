package com.irontigers.robot;

import java.util.Map;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Dashboard {
    public static enum TAB {
        SETUP("Setup"), AUTO("Autonomous"), TELEOP("Teleop"), DEBUG("Debug");

        private String name;

        public String getName() {
            return name;
        }

        TAB(String name) {
            this.name = name;
        }
    }

    private static Dashboard instance;

    private ShuffleboardTab setupTab;
    private ShuffleboardTab autoTab;
    private ShuffleboardTab teleopTab;
    // private ShuffleboardTab debugTab;

    private Dashboard() {
        VideoSource stream = new HttpCamera("Camera", "http://10.41.76.47:5800", HttpCameraKind.kMJPGStreamer);

        setupTab = Shuffleboard.getTab(TAB.SETUP.getName());
        setupTab.add(stream)
                .withPosition(4, 0)
                .withSize(6, 3)
                .withProperties(Map.of("Show crosshair", false, "show controls", false));

        autoTab = Shuffleboard.getTab(TAB.AUTO.getName());
        autoTab.add(stream)
               .withPosition(4, 0)
               .withSize(6, 4)
               .withProperties(Map.of("Show crosshair", false, "show controls", false));

        teleopTab = Shuffleboard.getTab(TAB.TELEOP.getName());
        teleopTab.add(stream)
                 .withPosition(4, 0)
                 .withSize(6, 4)
                 .withProperties(Map.of("Show crosshair", false, "show controls", false));

        // debugTab = Shuffleboard.getTab(TAB.DEBUG.getName());
    }

    public static Dashboard getInstance() {
        if (instance == null) {
            instance = new Dashboard();
        }

        return instance;
    }

    public void setActiveTab(TAB tab) {
        Shuffleboard.selectTab(tab.getName());
    }

    public ShuffleboardTab getTab(TAB tab) {
        switch (tab) {
            case SETUP:
                return Shuffleboard.getTab(TAB.SETUP.getName());
            case AUTO:
                return Shuffleboard.getTab(TAB.AUTO.getName());
            case TELEOP:
                return Shuffleboard.getTab(TAB.TELEOP.getName());
            case DEBUG:
                return Shuffleboard.getTab(TAB.DEBUG.getName());
            default:
                return null;
        }
    }
}