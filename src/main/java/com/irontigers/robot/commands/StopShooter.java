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

public class StopShooter extends CommandBase {
  private ShooterSystem shooterSys;
  /**
   * Creates a new StopShooter.
   */
  public StopShooter(ShooterSystem shooterSys) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSys = shooterSys;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSys.setFlywheelPower(shooterSys.getFlywheelPower() - Shooter.POWER_INCREMENT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSys.stopFlywhel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterSys.getFlywheelPower() <= 0;
  }
}
