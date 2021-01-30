/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.irontigers.robot.commands;

import com.irontigers.robot.Constants.Shooter;
import com.irontigers.robot.subsystems.ShooterSystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunShooterAtSpeed extends CommandBase {
  private ShooterSystem shooterSys;
  private double targetSpeed;

  /**
   * Creates a new RunShooter.
   */
  public RunShooterAtSpeed(ShooterSystem shooterSys, double targetSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSys = shooterSys;
    this.targetSpeed = targetSpeed;
    addRequirements(shooterSys);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedError = Math.abs(targetSpeed - shooterSys.getFlywheelRPS());
    double deltaPowScalar = speedError > 10.0 ? 1.0 : speedError / 10.0;

    if (targetSpeed - shooterSys.getFlywheelRPS() > 0) {
      shooterSys.setFlywheelPower(shooterSys.getFlywheelPower() + (Shooter.POWER_INCREMENT * deltaPowScalar));
    } else {
      shooterSys.setFlywheelPower(shooterSys.getFlywheelPower() - (Shooter.POWER_INCREMENT * deltaPowScalar));
    }
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
