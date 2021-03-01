/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.irontigers.robot;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Drive {
        public static final int FRNT_LFT = 2;
        public static final int BCK_LFT = 1;
        public static final int FRNT_RT = 4;
        public static final int BCK_RT = 3;

        public static final double ENC_CNV_FCTR = 1 / 8.45 * (Units.inchesToMeters(6) * Math.PI);
        public static final double ENC_CNV_FCTR_V = ENC_CNV_FCTR / 60;
    }

    public static final class Shooter {
        public static final int FLYWHEEL_ADDR = 1;
        public static final int TURRET_ADDR = 2;

        public static final double DEFUALT_SHOOTER_SPD = 1;
        public static final double DEFAULT_TURRET_SPD = 0.05;
        public static final double AUTO_TURRET_SPD_FAST = 0.1;
        public static final double AUTO_TURRET_SPD_SLOW = 0.07;
        public static final double MAX_TURRET_SPD = 0.1;
        public static final double TURRET_ANGLE_CNV_FACTOR = 204800.0 / 1803.0;
        public static final int MAX_TURRET_ANGLE = 30; // Final value is 85 (we hope)

        public static final int MOTOR_TOOTH_CNT = 16;
        public static final int FLYWHEEL_TOOTH_CNT = 24;
        public static final double FLYWHEEL_CIRCUM_FT = 0.5 * Math.PI;
        public static final double FLYWHEEL_RPS_CNV_FACTOR = (16.0 * 10.0) / (24.0 * 2048.0);


        public static final double TARGET_FLYWHEEL_RPM = 67000;

        public static final double ANGLE = Units.degreesToRadians(30);
        public static final double HEIGHT_OFFSET = 0; //3.75;
        public static final double TARGET_HEIGHT = GameField.TOP_PORT_CENTER_HEIGHT - HEIGHT_OFFSET;

        public static final double POWER_INCREMENT = 1.0 / 100.0;
        public static final long TURRET_ENC_OFFSET = -527;

        public static final double TURRET_KP = 0.00413;
        public static final double TURRET_KI = 0.00;
        public static final double TURRET_KD = 0.0121; 

        public static final double TURRET_MAX_V = 100;
        public static final double TURRET_MAX_A = 100;

        public static final double TURRET_KS = 0.37;
        public static final double TURRET_KV = 0.00863;
    }

    public static final class Magazine {
        public static final int MAG_ADDR = 6;
        public static final int INTAKE_ADDR = 7;

        public static final int BOT_SENSOR_PORT = 0;
        public static final int TOP_SENSOR_PORT = 1;

        public static final int GATE_PORT = 0;

        public static final double MAG_SPD = .2;
        public static final double INTAKE_SPD = .3;
    }

    public static final class VISION {
        public static final double HEIGHT = 37.5;
        private static final double yOffset = 0.830619;
    }

    public static final class Controllers {
        public static final int PORT = 0;
        public static final int TEST_PORT = 1;
    }

    public static final class GameField {
        public static final double TOP_PORT_CENTER_HEIGHT = 8.1875 * 12.0;
    }

    public static final class Falcon500 {
        public static final int ENCODER_PPR = 2048;
    }

    public static final class Characterization {
        
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(FeedForward.TRACK_WIDTH);
        public static final double kPDriveVel = FeedBack.KP;

        
        public static final class FeedForward {
            public static final double TRACK_WIDTH = 2.6605493765160295;
            public static final double KS = 0.302;
            public static final double KV = 0.0326;
            public static final double KA = 0.00655;
        }

        public static final class FeedBack {
            public static final double KV = FeedForward.KV;
            public static final double KS = FeedForward.KS;
            public static final double KA = FeedForward.KA;
            public static final double KP = 0.311;
            public static final double KD = 0.0;
        }

        
    }
}
