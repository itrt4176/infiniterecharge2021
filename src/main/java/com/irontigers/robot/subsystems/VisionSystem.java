/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.irontigers.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.irontigers.robot.Constants.VISION;

// import com.irontigers.robot.LedController;

import static com.irontigers.robot.Constants.GameField;

public class VisionSystem extends SubsystemBase {
  private NetworkTable limelight;
  // private LedController ledRing;

  private Boolean seesTarget = false;
  private double xAngle;
  private double yAngle;
  private Boolean ledsEnabled = false;

  /**
   * Creates a new VisionSystem.
   */
  public VisionSystem() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    // ledRing = new LedController(1);
    enableLeds();
  }

  public Boolean seesTarget() {
    return seesTarget;
  }

  public double getXAngle() {
    return xAngle;
  }

  public double getDistanceToTarget() {
    if (seesTarget()) {
      return (GameField.TOP_PORT_CENTER_HEIGHT - VISION.HEIGHT) / Math.tan(Math.toRadians(yAngle));
    } else {
      return -1;
    }
  }

  public void enableLeds() {
    limelight.getEntry("ledMode").setNumber(3);
    // ledRing.enableLeds();
  }

  public void disableLeds() {
    limelight.getEntry("ledMode").setNumber(1);
    // ledRing.disableLeds();
  }

  public void setToDriving() {
    limelight.getEntry("camMode").setNumber(1);
  }

  public void setToVision() {
    limelight.getEntry("camMode").setNumber(0);
  }

  @Override
  public void periodic() {
    seesTarget = limelight.getEntry("tv").getDouble(0) == 1;
    xAngle = limelight.getEntry("tx").getDouble(0);
    yAngle = limelight.getEntry("ty").getDouble(0);
    

    SmartDashboard.putNumber("LimeLightDistanceInches", getDistanceToTarget());
    SmartDashboard.putBoolean("Limelight Sees Target", seesTarget);
    SmartDashboard.putNumber("Limelight X Angle", xAngle);
    SmartDashboard.putNumber("Limelight Y Angle", yAngle);
  }
}
