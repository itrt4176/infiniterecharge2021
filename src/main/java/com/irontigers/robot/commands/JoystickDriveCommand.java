/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.irontigers.robot.commands;

import com.irontigers.robot.util.Utils;
import com.irontigers.robot.subsystems.DriveSystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class JoystickDriveCommand extends CommandBase {
  private DriveSystem driveSys;
  private XboxController controller;

  /**
   * Creates a new JoystickDriveCommand.
   */
  public JoystickDriveCommand(DriveSystem driveSys, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSys = driveSys;
    this.controller = controller;
    addRequirements(driveSys);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSys.drive(Utils.deadZone(controller.getY(Hand.kLeft)), Utils.deadZone(controller.getX(Hand.kLeft)));
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
