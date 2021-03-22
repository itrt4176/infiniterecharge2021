/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.irontigers.robot.subsystems;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.irontigers.robot.Constants.Shooter;
import com.irontigers.robot.util.InterpolatingDouble;
import com.irontigers.robot.util.InterpolatingTreeMap;

import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSystem extends SubsystemBase {
  private WPI_TalonFX flywheelMotor;
  private WPI_TalonFX turretMotor;
  private LinearFilter flywheelRPMFilter;
  private SimpleMotorFeedforward flywheelFF;

  private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> shooterSpeedMap;

  private double targetSpeed;
  
  /**
   * Creates a new ExampleSubsystem.
   */
  public ShooterSystem() {
    flywheelMotor = new WPI_TalonFX(Shooter.FLYWHEEL_ADDR);
    flywheelMotor.setInverted(true);
    flywheelMotor.setSelectedSensorPosition(0);
    flywheelFF = new SimpleMotorFeedforward(0.00021, 0.173, 0.0149);

    turretMotor = new WPI_TalonFX(Shooter.TURRET_ADDR);
    turretMotor.setInverted(true);
    turretMotor.setSelectedSensorPosition(0);
    // turretMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    flywheelRPMFilter = LinearFilter.movingAverage(20);

    shooterSpeedMap = new InterpolatingTreeMap<>(7);
    shooterSpeedMap.put(new InterpolatingDouble(-1.0), new InterpolatingDouble(8.0));
    shooterSpeedMap.put(new InterpolatingDouble(97.0), new InterpolatingDouble(60.1));
    shooterSpeedMap.put(new InterpolatingDouble(99.0), new InterpolatingDouble(57.7));
    shooterSpeedMap.put(new InterpolatingDouble(118.0), new InterpolatingDouble(56.1));
    shooterSpeedMap.put(new InterpolatingDouble(124.0), new InterpolatingDouble(56.98));
    shooterSpeedMap.put(new InterpolatingDouble(183.0), new InterpolatingDouble(59.2));
    shooterSpeedMap.put(new InterpolatingDouble(191.0), new InterpolatingDouble(59.3));
    shooterSpeedMap.put(new InterpolatingDouble(241.6), new InterpolatingDouble(62.6));

    targetSpeed = 60.1;
  }

  /**
   * Sets the percent power output of the shooter's flywheel motor.
   * 
   * @param power The power to set. Value should be between -1.0 (100% power
   *              backwards) and 1.0 (100% power forwards).
   */
  public void setFlywheelPower(double power) {
    flywheelMotor.set(Util.cap(power, 1));
  }

  public void setFlywheelVoltage(double volts) {
    flywheelMotor.setVoltage(volts);
  }

  public double getFlywheelPower() {
    return flywheelMotor.get();
  }

  /**
   * Sets the percent power output of the shooter's turret motor.
   * 
   * @param power The power to set. Value should be between -0.1 (10% power
   *              backwards) and 0.1 (10% power forwards).
   */
  public void setTurretPower(double power) {
    turretMotor.set(Util.cap(power, Shooter.MAX_TURRET_SPD));
  }

  public void setTurretVoltage(double volts) {
    turretMotor.setVoltage(volts);
  }

  public double getTurretPower() {
    return turretMotor.get();
  }

  public double getFlywheelRPS() {
    double rps = flywheelMotor.getSelectedSensorVelocity() * Shooter.FLYWHEEL_RPS_CNV_FACTOR;
    return flywheelRPMFilter.calculate(rps);
  }

  public double getTurretAngle() {
    return turretMotor.getSelectedSensorPosition() / Shooter.TURRET_ANGLE_CNV_FACTOR;
    // return turretMotor.getSelectedSensorPosition();
  }

  public void stopFlywhel() {
    setFlywheelPower(0);
  }

  public void stopTurret() {
    setTurretPower(0);
  }

  public boolean isReadyToShoot() {
    return targetSpeed - getFlywheelRPS() < 1.2;
  }

  public void setTargetRPS(double shooterSpeed) {
    targetSpeed = shooterSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Shooter Power", getFlywheelPower());
    SmartDashboard.putNumber("Shooter Speed", getFlywheelRPS());
    SmartDashboard.putBoolean("Ready to shoot", isReadyToShoot());
    SmartDashboard.putNumber("Turret Angle", getTurretAngle());

    SmartDashboard.putNumber("Speed for distance", targetSpeed);
  }

  public double getSpeedForDist(double distance) {
    return shooterSpeedMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }

  public SimpleMotorFeedforward getFlywheelFF() {
    return flywheelFF;
  }

  public double getTargetRPS() {
    return targetSpeed;
  }
}
