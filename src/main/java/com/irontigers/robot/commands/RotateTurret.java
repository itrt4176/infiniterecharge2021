/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.irontigers.robot.commands;

import static com.irontigers.robot.Constants.Shooter;

import com.irontigers.robot.RobotContainer;
import com.irontigers.robot.subsystems.ShooterSystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateTurret extends CommandBase {
  public static enum Direction {
    LEFT, RIGHT
  }

  private ShooterSystem shooterSys;
  private Direction direction;

  /**
   * Creates a new RotateTurret.
   */
  public RotateTurret(ShooterSystem shooterSys, Direction direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSys);
    this.shooterSys = shooterSys;
    this.direction = direction;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (direction) {
      case RIGHT:
        shooterSys.setTurretPower(Shooter.DEFAULT_TURRET_SPD);        
        break;
      case LEFT:
        shooterSys.setTurretPower(-Shooter.DEFAULT_TURRET_SPD);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSys.stopTurret();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch (direction) {
    case RIGHT:
      return !(shooterSys.getTurretAngle() < Shooter.MAX_TURRET_ANGLE);
    case LEFT:
      return !(-shooterSys.getTurretAngle() < Shooter.MAX_TURRET_ANGLE);
    default:
      return true;
    }
  }
}
