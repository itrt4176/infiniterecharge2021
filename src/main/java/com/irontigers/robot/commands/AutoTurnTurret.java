/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.irontigers.robot.commands;

import com.irontigers.robot.subsystems.DriveSystem;
import com.irontigers.robot.subsystems.ShooterSystem;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoTurnTurret extends CommandBase {
  /**
   * Creates a new AutoTurnTurret.
   */
  private DriveSystem driveSys;
  private Pose2d currentLoc;
  private Pose2d targetLoc;
  private ShooterSystem shooterSys;
  private double targetAngle;

  public AutoTurnTurret(DriveSystem driveSys, ShooterSystem shooterSys) {
    this.driveSys = driveSys;
    this.shooterSys = shooterSys;
    addRequirements(shooterSys);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentLoc = driveSys.getRobotPosition();
    targetLoc = new Pose2d(Units.feetToMeters(52.4375), Units.feetToMeters(18.645), Rotation2d.fromDegrees(0));
    Pose2d poseDifference = currentLoc.relativeTo(targetLoc);
    targetAngle = Math.atan2(poseDifference.getTranslation().getY(), poseDifference.getTranslation().getX());
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // BANG-BANG Control
      shooterSys.setTurretPower(.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSys.setTurretPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     if (shooterSys.getTurretAngle() < targetAngle - 3.0) { // 3 degree deadband, should make it a constant
      return true;
     } else {
       return false;
     }
  }
}
