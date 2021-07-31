/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.irontigers.robot.commands;

import static com.irontigers.robot.Constants.Shooter;

import com.irontigers.robot.Dashboard;
import com.irontigers.robot.Dashboard.TAB;
import com.irontigers.robot.subsystems.DriveSystem;
import com.irontigers.robot.subsystems.ShooterSystem;
import com.irontigers.robot.subsystems.VisionSystem;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class VisionAim extends CommandBase {
  public static enum Direction {
    LEFT, RIGHT
  }

  private ShooterSystem shooterSys;
  private VisionSystem visionSys;
  private double setpoint;
  private boolean targetDetected;
  private boolean turnTurret;

  /**
   * Creates a new RotateTurret.
   */
  public VisionAim(ShooterSystem shooterSys, VisionSystem visionSys) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSys);
    this.shooterSys = shooterSys;
    this.visionSys = visionSys;
  }

  @Override
  public void initialize() {
    // visionSys.enableLeds();
    visionSys.setToVision();
    targetDetected = visionSys.seesTarget();
    setpoint = shooterSys.getTurretAngle() + visionSys.getXAngle();
    turnTurret = true;
    // Dashboard.getInstance().getTab(TAB.TELEOP).addBoolean("Aiming", () -> {
    //   return true;
    // }).withWidget(BuiltInWidgets.kBooleanBox).withPosition(2, 3).withSize(1, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putBoolean("AutoAim Target", true);
    SmartDashboard.putNumber("AutoAim Target", setpoint);
    SmartDashboard.putNumber("AutoAim Angle", shooterSys.getTurretAngle());

    double speed = Math.abs(setpoint - shooterSys.getTurretAngle()) < 7.5
        ? (0.033 * (Math.abs(setpoint - shooterSys.getTurretAngle()) / 7.5)) + 0.067
        : Shooter.AUTO_TURRET_SPD_FAST;

    if (turnTurret) {
      if (setpoint > shooterSys.getTurretAngle()) {
        shooterSys.setTurretPower(speed);
      } else {
        shooterSys.setTurretPower(-speed);
      }
    } else {
      shooterSys.stopTurret();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSys.stopTurret();

    // Dashboard.getInstance().getTab(TAB.TELEOP).addBoolean("Aiming", () -> {
    //   return false;
    // }).withWidget(BuiltInWidgets.kBooleanBox).withPosition(2, 3).withSize(1, 1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (!targetDetected)
    // return true;

    if (Math.abs(shooterSys.getTurretAngle()) >= Shooter.MAX_TURRET_ANGLE) {
      return true;
    }

    if (Math.abs(setpoint - shooterSys.getTurretAngle()) < 0.21) { // 0.22 degree deadband, should make it a constant
      turnTurret = false;
      if (Math.abs(visionSys.getXAngle()) < 0.21) {
        return true;
      } else {
        setpoint = shooterSys.getTurretAngle() + visionSys.getXAngle();
        turnTurret = true;
        return false;
      }
      // return true;
    } else {
      turnTurret = true;
      return false;
    }
  }
}
