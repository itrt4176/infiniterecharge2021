/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.irontigers.robot;

import static edu.wpi.first.wpilibj.XboxController.Button.kA;
import static edu.wpi.first.wpilibj.XboxController.Button.kB;
import static edu.wpi.first.wpilibj.XboxController.Button.kBack;
import static edu.wpi.first.wpilibj.XboxController.Button.kBumperRight;
import static edu.wpi.first.wpilibj.XboxController.Button.kBumperLeft;
import static edu.wpi.first.wpilibj.XboxController.Button.kStart;
import static edu.wpi.first.wpilibj.XboxController.Button.kX;
import static edu.wpi.first.wpilibj.XboxController.Button.kY;

import com.irontigers.robot.Constants.Controllers;
import com.irontigers.robot.commands.AutonomousDrive;
import com.irontigers.robot.commands.JoystickDriveCommand;
import com.irontigers.robot.commands.RotateTurret;
import com.irontigers.robot.commands.RunShooter;
import com.irontigers.robot.commands.Shoot;
import com.irontigers.robot.commands.StopShooter;
import com.irontigers.robot.commands.VisionAim;
import com.irontigers.robot.subsystems.CorrectXboxController;
import com.irontigers.robot.subsystems.DriveSystem;
import com.irontigers.robot.subsystems.MagazineSystem;
import com.irontigers.robot.subsystems.ShooterSystem;
import com.irontigers.robot.subsystems.VisionSystem;
import com.irontigers.robot.triggers.BallPresenceTrigger;
import com.irontigers.robot.triggers.DPadButton;
import com.irontigers.robot.triggers.DPadButton.DPadDirection;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private CorrectXboxController controller = new CorrectXboxController(Controllers.PORT);
  private CorrectXboxController testController = new CorrectXboxController(Controllers.TEST_PORT);

  private JoystickButton shootAllButton = new JoystickButton(controller, kA.value);
  private JoystickButton shootOneButton = new JoystickButton(controller, kB.value);
  private JoystickButton cancelShootingButton = new JoystickButton(controller, kY.value);
  private JoystickButton autoAimButton = new JoystickButton(controller, kBumperRight.value);

  private JoystickButton incrementCountButton = new JoystickButton(controller, kStart.value);
  private JoystickButton decrementCountButton = new JoystickButton(controller, kBack.value);
  //
  //
  //
  //

  //
  //
  //
  //
  private DPadButton turretLeftButton = new DPadButton(controller, DPadDirection.LEFT);
  private DPadButton turretRightButton = new DPadButton(controller, DPadDirection.RIGHT);

  private JoystickButton startAutonomousButton = new JoystickButton(controller, kBack.value);

  private DriveSystem driveSystem = new DriveSystem();
  private ShooterSystem shooterSystem = new ShooterSystem();
  private MagazineSystem magSystem = new MagazineSystem();
  private VisionSystem visionSystem = new VisionSystem();

  private JoystickDriveCommand joyDrive = new JoystickDriveCommand(driveSystem, controller);
  private AutonomousDrive autonomousDrive = new AutonomousDrive(driveSystem);
  private BallPresenceTrigger topBallSensor = new BallPresenceTrigger(magSystem.getTopBallSensor());
  private BallPresenceTrigger bottomBallSensor = new BallPresenceTrigger(magSystem.getBottomBallSensor());
  private MedianFilter bottomSensorFilter = new MedianFilter(5);

  private ConditionalCommand intakeSwitchCommand = new ConditionalCommand(new SequentialCommandGroup( // onTrue
      new InstantCommand(magSystem::disableIntake, magSystem),
      new InstantCommand(magSystem::disableMagazine, magSystem)),
      new SequentialCommandGroup( // onFalse
          new InstantCommand(magSystem::enableMagazine, magSystem),
          new InstantCommand(magSystem::enableIntake, magSystem)),
      magSystem::isMagFull);

  private SequentialCommandGroup shootOnceCommand = new SequentialCommandGroup(
      new InstantCommand(visionSystem::setToVision), new VisionAim(shooterSystem, visionSystem),
      new Shoot(magSystem, shooterSystem, visionSystem, topBallSensor),
      new InstantCommand(magSystem::closeGate, magSystem), new WaitCommand(0.5), new StopShooter(shooterSystem),
      new InstantCommand(visionSystem::setToDriving));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    driveSystem.setDefaultCommand(joyDrive);

    intakeSwitchCommand.initialize();
    magSystem.setDefaultCommand(intakeSwitchCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    shootOneButton.whenPressed(shootOnceCommand);
    shootAllButton.whenPressed(getShootAllCommand());

    incrementCountButton.whenPressed(new InstantCommand(magSystem::incrementBalls));
    decrementCountButton.whenPressed(new InstantCommand(magSystem::decrementBalls));

    turretLeftButton.whenHeld(new RotateTurret(shooterSystem, RotateTurret.Direction.LEFT));
    turretRightButton.whenHeld(new RotateTurret(shooterSystem, RotateTurret.Direction.RIGHT));

    // bottomBallSensor.whenInactive(magSystem::incrementBalls, magSystem);
  }

  public boolean isTurretLeftButtonPressed() {
    return turretLeftButton.get();
  }

  public boolean isTurretRightButtonPressed() {
    return turretRightButton.get();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new SequentialCommandGroup(
      new InstantCommand(visionSystem::setToVision),
      new InstantCommand(magSystem::closeGate), new AutonomousDrive(driveSystem),
        new InstantCommand(visionSystem::enableLeds), new WaitUntilCommand(visionSystem::seesTarget),
        new VisionAim(shooterSystem, visionSystem), getShootAllCommand(), new InstantCommand(visionSystem::disableLeds),
        new InstantCommand(visionSystem::setToDriving));
  }

  public void initTesting() {
    // visionSystem.disableLeds();
    visionSystem.enableLeds();
    visionSystem.setToVision();
    JoystickButton enableShooterButton = new JoystickButton(testController, kX.value);
    JoystickButton disableMagButton = new JoystickButton(testController, kY.value);
    JoystickButton shootButton = new JoystickButton(testController, kA.value);
    JoystickButton stopShooterButton = new JoystickButton(testController, kB.value);
    JoystickButton increaseFlywheelButton = new JoystickButton(testController, kStart.value);
    JoystickButton decreaseFlywheelButton = new JoystickButton(testController, kBack.value);
    JoystickButton opengateButton = new JoystickButton(testController, kBumperRight.value);
    JoystickButton closegateButton = new JoystickButton(testController, kBumperLeft.value);

    enableShooterButton.whenPressed(() -> shooterSystem.setFlywheelPower(0.2), shooterSystem);
    disableMagButton.whenPressed(magSystem::disableMagazine, magSystem);

    shootButton.whenPressed(new RunShooter(shooterSystem));
    stopShooterButton.whenPressed(new StopShooter(shooterSystem));

    shooterSystem.setFlywheelPower(.2);

    increaseFlywheelButton.whenPressed(() -> shooterSystem.setFlywheelPower(shooterSystem.getFlywheelPower() + 0.025),
        shooterSystem);
    decreaseFlywheelButton.whenPressed(() -> shooterSystem.setFlywheelPower(shooterSystem.getFlywheelPower() - 0.025),
        shooterSystem);

    opengateButton.whenPressed(new InstantCommand(magSystem::openGate, magSystem));
    closegateButton.whenPressed(new InstantCommand(magSystem::closeGate, magSystem));
  }

  private Command getShootAllCommand() {
    SequentialCommandGroup shootAllCommand = new SequentialCommandGroup(new VisionAim(shooterSystem, visionSystem));
    for (int i = 0; i < magSystem.getStoredBalls(); i++) {
      shootAllCommand = shootAllCommand.andThen(new Shoot(magSystem, shooterSystem, visionSystem, topBallSensor));
    }

    return shootAllCommand.andThen(new InstantCommand(magSystem::closeGate, magSystem), new WaitCommand(0.5),
        new StopShooter(shooterSystem), new InstantCommand(visionSystem::setToDriving));

  }

  public VisionSystem getVisionSystem() {
    return visionSystem;
  }
}