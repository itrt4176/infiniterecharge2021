// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.irontigers.robot.sim;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpiutil.math.MathUtil;


public class DifferentialDriveCompat extends DifferentialDrive {
    public static final double kDefaultQuickStopThreshold = 0.2;
    public static final double kDefaultQuickStopAlpha = 0.1;

    private final SpeedController m_leftMotor;
    private final SpeedController m_rightMotor;

    private double m_rightSideInvertMultiplier = -1.0;
    private boolean m_reported;

    private double leftMotorOutput;
    private double rightMotorOutput;

    /**
     * Construct a DifferentialDrive.
     *
     * <p>
     * To pass multiple motors per side, use a {@link SpeedControllerGroup}. If a
     * motor needs to be inverted, do so before passing it in.
     */
    public DifferentialDriveCompat(SpeedController leftMotor, SpeedController rightMotor) {
        super(leftMotor, rightMotor);
        m_leftMotor = leftMotor;
        m_rightMotor = rightMotor;
    }

    @Override
    public void arcadeDrive(double xSpeed, double zRotation) {
        arcadeDrive(xSpeed, zRotation, true);
    }
    
    @Override
    public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
        if (!m_reported) {
            HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDrive2_DifferentialArcade, 2);
            m_reported = true;
        }

        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        xSpeed = applyDeadband(xSpeed, m_deadband);

        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
        zRotation = applyDeadband(zRotation, m_deadband);

        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        if (squareInputs) {
            xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
            zRotation = Math.copySign(zRotation * zRotation, zRotation);
        }

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            } else {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            }
        }

        leftMotorOutput = MathUtil.clamp(leftMotorOutput, -1.0, 1.0) * m_maxOutput;
        m_leftMotor.set(leftMotorOutput);

        double maxOutput = m_maxOutput * m_rightSideInvertMultiplier;
        rightMotorOutput = MathUtil.clamp(rightMotorOutput, -1.0, 1.0) * maxOutput;
        m_rightMotor.set(rightMotorOutput);

        feed();
    }

    public double getLeftMotorOutput() {
        return leftMotorOutput;
    }

    public double getRightMotorOutput() {
        return rightMotorOutput;
    }
}
