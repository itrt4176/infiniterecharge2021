/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.irontigers.robot.commands;

import com.irontigers.robot.subsystems.MagazineSystem;
import com.irontigers.robot.subsystems.ShooterSystem;
import com.irontigers.robot.subsystems.VisionSystem;
import com.irontigers.robot.triggers.BallPresenceTrigger;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Shoot extends SequentialCommandGroup {

  /**
   * Creates a new Shoot.
   */
  public Shoot(MagazineSystem magSystem, ShooterSystem shooterSystem, VisionSystem visionSys, BallPresenceTrigger topBallSensor) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new InstantCommand(visionSys::setToVision),
      new WaitCommand(0.2),
      new InstantCommand(magSystem::disableIntake, magSystem),
      new InstantCommand(magSystem::closeGate),
      new ParallelRaceGroup(
        new SequentialCommandGroup(
          new InstantCommand(() -> shooterSystem.setTargetRPS(shooterSystem.getSpeedForDist(visionSys.getDistanceToTarget()))),
          new RunShooterAtSpeed(shooterSystem, shooterSystem.getTargetRPS())
        ),
        new SequentialCommandGroup(
          new WaitUntilCommand(shooterSystem::isReadyToShoot),
          new InstantCommand(magSystem::enableMagazine, magSystem),
          new InstantCommand(magSystem::openGate, magSystem),
          new WaitUntilCommand(() -> {
            return !shooterSystem.isReadyToShoot();
          }),
          new InstantCommand(magSystem::decrementBalls, magSystem)
        )
      )
    );
  }

  /**
   * Creates a new Shoot.
   */
  // public Shoot(MagazineSystem magSystem, ShooterSystem shooterSystem, VisionSystem visionSys) {
  //   // Add your commands in the super() call, e.g.
  //   // super(new FooCommand(), new BarCommand());
  //   super(
  //     new InstantCommand(magSystem::disableIntake, magSystem),
  //     new ParallelCommandGroup(
  //       new RunShooterPID(shooterSystem, shooterSystem.getSpeedForDist(visionSys.getDistanceToTarget())),
  //       new SequentialCommandGroup(
  //       new InstantCommand(magSystem::enableMagazine, magSystem),
  //       new InstantCommand(magSystem::openGate, magSystem), new WaitUntilCommand(() -> {
  //         return !shooterSystem.isReadyToShoot();
  //       }), new InstantCommand(magSystem::decrementBalls, magSystem)
  //     )
  //     )
  //   );
  // }
}
