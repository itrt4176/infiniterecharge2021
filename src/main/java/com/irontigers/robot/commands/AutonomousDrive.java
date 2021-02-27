/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.irontigers.robot.commands;


import com.irontigers.robot.subsystems.DriveSystem;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;



public class AutonomousDrive extends CommandBase {
  private DriveSystem driveSys;
  private Pose2d currentPos;
  private Pose2d endPos;

  private double distanceError;
  
  /**
   * Creates a new AutonomousDrive.
   */
  public AutonomousDrive(DriveSystem driveSys) {
    this.driveSys = driveSys;
    addRequirements(driveSys);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSys.initStartingPose();
    currentPos = driveSys.getRobotPosition();
    endPos = currentPos.plus(new Transform2d(new Translation2d(Units.feetToMeters(-4), 0), Rotation2d.fromDegrees(0)));
    distanceError = currentPos.relativeTo(endPos).getTranslation().getX();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPos = driveSys.getRobotPosition();
    distanceError = currentPos.relativeTo(endPos).getTranslation().getX();
    driveSys.drive(-0.3, 0);
    SmartDashboard.putNumber("Auto dist error", distanceError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    driveSys.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (distanceError >= 0) {
      return false;
    } else {
      return true;
    }
  }
}
