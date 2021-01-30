/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.irontigers.robot.commands;

import com.irontigers.robot.subsystems.MagazineSystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeControl extends CommandBase {
  MagazineSystem magSys;
  /**
   * Creates a new IntakeControl.
   */
  public IntakeControl(MagazineSystem magSys) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.magSys = magSys;
    addRequirements(magSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (magSys.getStoredBalls() < 3) {
      magSys.enableIntake();
    } else {
      magSys.disableIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
