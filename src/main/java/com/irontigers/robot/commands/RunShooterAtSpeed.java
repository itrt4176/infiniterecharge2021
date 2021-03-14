/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.irontigers.robot.commands;

import com.irontigers.robot.Constants.Shooter;
import com.irontigers.robot.subsystems.ShooterSystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunShooterAtSpeed extends CommandBase {
  private ShooterSystem shooterSys;
  private double targetSpeed;

  public static double settingFlyWheelSpeed = 7.5;
  /**
   * Creates a new RunShooter.
   */

  public RunShooterAtSpeed(ShooterSystem shooterSys, double targetSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSys = shooterSys;
    this.targetSpeed = targetSpeed;
    addRequirements(shooterSys);
  }

  public static void setSpeedtoSixty() {
    settingFlyWheelSpeed = 60;
  }

  public static void setSpeedtoSeven() {
    settingFlyWheelSpeed = 7.5;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedError = Math.abs(shooterSys.getTargetRPS() - shooterSys.getFlywheelRPS());
    double deltaPowScalar = speedError > 10.0 ? 1.0 : speedError / 10.0;

    if (shooterSys.getTargetRPS() - shooterSys.getFlywheelRPS() > 0) {
      shooterSys.setFlywheelPower(shooterSys.getFlywheelPower() + (Shooter.POWER_INCREMENT * deltaPowScalar));
    } else {
      shooterSys.setFlywheelPower(shooterSys.getFlywheelPower() - (Shooter.POWER_INCREMENT * deltaPowScalar));
    }

    SmartDashboard.putNumber("Target Speed", targetSpeed);
    SmartDashboard.putNumber("SpeedError", speedError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return shooterSys.getFlywheelPower() == 1 || shooterSys.isReadyToShoot();
    return false;
  }
}
