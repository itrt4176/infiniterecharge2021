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

import java.util.List;
import java.util.function.BiConsumer;

import com.irontigers.robot.Constants.Controllers;
import com.irontigers.robot.commands.AutonomousDrive;
import com.irontigers.robot.commands.JoystickDriveCommand;
import com.irontigers.robot.commands.RotateTurret;
import com.irontigers.robot.commands.RunShooter;
import com.irontigers.robot.commands.RunShooterAtSpeed;
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

// import org.graalvm.compiler.lir.amd64.vector.AMD64VectorShuffle.ConstShuffleBytesOp;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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
  public boolean isSmallNum;

  private JoystickButton incrementCountButton = new JoystickButton(controller, kStart.value);
  private JoystickButton decrementCountButton = new JoystickButton(controller, kBack.value);

  JoystickButton opengateButton = new JoystickButton(testController, kBumperRight.value);
  JoystickButton closegateButton = new JoystickButton(testController, kBumperLeft.value);
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

  private DPadButton powPortForwardButton = new DPadButton(controller, DPadDirection.UP);
  private DPadButton powPortBackwardButton = new DPadButton(controller, DPadDirection.DOWN);

  // private JoystickButton startAutonomousButton = new JoystickButton(controller, kBack.value);

  private DriveSystem driveSystem = new DriveSystem();
  private ShooterSystem shooterSystem = new ShooterSystem();
  private MagazineSystem magSystem = new MagazineSystem();
  private VisionSystem visionSystem = new VisionSystem();

  private JoystickDriveCommand joyDrive = new JoystickDriveCommand(driveSystem, controller);
  private AutonomousDrive autonomousDrive = new AutonomousDrive(driveSystem);
  private BallPresenceTrigger topBallSensor = new BallPresenceTrigger(magSystem.getTopBallSensor());

  private BallPresenceTrigger bottomBallSensor = new BallPresenceTrigger(magSystem.getBottomBallSensor());
  private MedianFilter bottomSensorFilter = new MedianFilter(5);

  // private JoystickButton increaseFlywheelButton = new JoystickButton(controller, kStart.value);
  // private JoystickButton decreaseFlywheelButton = new JoystickButton(controller, kBack.value);
    

  private ConditionalCommand intakeSwitchCommand = new ConditionalCommand(new SequentialCommandGroup( // onTrue
      new InstantCommand(magSystem::disableIntake, magSystem),
      new InstantCommand(magSystem::disableMagazine, magSystem)),
      new SequentialCommandGroup( // onFalse
          new InstantCommand(magSystem::enableMagazine, magSystem),
          new InstantCommand(magSystem::enableIntake, magSystem)),
      magSystem::isMagFull);

  private SequentialCommandGroup shootOnceCommand = new SequentialCommandGroup(
      // new InstantCommand(visionSystem::setToVision), new VisionAim(shooterSystem, visionSystem),
      new Shoot(magSystem, shooterSystem, visionSystem, topBallSensor),
      new InstantCommand(magSystem::closeGate, magSystem), new WaitCommand(0.5), new StopShooter(shooterSystem),
      new InstantCommand(visionSystem::setToDriving));


  // private Command singleshot = new InstantCommand(shooterSystem::)
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
    shootAllButton.whenPressed(new SequentialCommandGroup(
          new InstantCommand(visionSystem::setToVision),
          new InstantCommand(magSystem::closeGate),
            new InstantCommand(visionSystem::enableLeds), new WaitUntilCommand(visionSystem::seesTarget),
            new VisionAim(shooterSystem, visionSystem), getShootAllCommand(), new InstantCommand(visionSystem::disableLeds),
            new InstantCommand(visionSystem::setToDriving)));

    incrementCountButton.whenPressed(new InstantCommand(magSystem::incrementBalls));
    decrementCountButton.whenPressed(new InstantCommand(magSystem::decrementBalls));

    turretLeftButton.whenHeld(new RotateTurret(shooterSystem, RotateTurret.Direction.LEFT));
    turretRightButton.whenHeld(new RotateTurret(shooterSystem, RotateTurret.Direction.RIGHT));

    opengateButton.whenPressed(new InstantCommand(magSystem::openGate, magSystem));
    closegateButton.whenPressed(new InstantCommand(magSystem::closeGate, magSystem));

    // increaseFlywheelButton.whenPressed(() -> shooterSystem.setTargetRPS(60),
    //     shooterSystem);
    // decreaseFlywheelButton.whenPressed(() -> shooterSystem.setTargetRPS(8),
    //     shooterSystem);

    bottomBallSensor.whenInactive(magSystem::incrementBalls);
    topBallSensor.whenInactive(magSystem::decrementBalls);

    powPortForwardButton.whenPressed(
      new AutonomousDrive(driveSystem, 
      new Transform2d(new Translation2d(Units.feetToMeters(13), 0), Rotation2d.fromDegrees(0)), 
      0.65, 
      -0.1)
    );

    powPortBackwardButton.whenPressed(
      new AutonomousDrive(driveSystem, 
      new Transform2d(new Translation2d(Units.feetToMeters(-13), 0), Rotation2d.fromDegrees(0)), 
      -0.65, 
      -0.1)
    );
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
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   return new SequentialCommandGroup(
  //     new InstantCommand(visionSystem::setToVision),
  //     new InstantCommand(magSystem::closeGate), new AutonomousDrive(driveSystem),
  //       new InstantCommand(visionSystem::enableLeds), new WaitUntilCommand(visionSystem::seesTarget),
  //       new VisionAim(shooterSystem, visionSystem), getShootAllCommand(), new InstantCommand(visionSystem::disableLeds),
  //       new InstantCommand(visionSystem::setToDriving));
  // }
//////////////////////////////////////////

 /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.Characterization.FeedForward.KS,
              Constants.Characterization.FeedForward.KV,
              Constants.Characterization.FeedForward.KA),
            Constants.Characterization.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(Constants.Characterization.kMaxSpeedMetersPerSecond,
                             Constants.Characterization.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.Characterization.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = DriveSystem.path("Debug");

    Trajectory mini = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), 
                                                            List.of(
                                                              new Translation2d(1, 0), new Translation2d(2, 0)), 
                                                              new Pose2d(2, 0, new Rotation2d(0)), 
                                                            config);


    BiConsumer<Double, Double> outVolts = (l, r) -> driveSystem.tankDriveVolts(l, r);

    RamseteCommand ramseteCommand = new RamseteCommand(
        // exampleTrajectory,
        mini,
        driveSystem.getPose2d(),
        new RamseteController(Constants.Characterization.kRamseteB, Constants.Characterization.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.Characterization.FeedForward.KS,
                                   Constants.Characterization.FeedForward.KV,
                                   Constants.Characterization.FeedForward.KA),
        Constants.Characterization.kDriveKinematics,
        driveSystem.getWheelSpeeds(),
        new PIDController(Constants.Characterization.kPDriveVel, 0, 0),
        new PIDController(Constants.Characterization.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        outVolts,
        driveSystem
    );

    // Reset odometry to the starting pose of the trajectory.
    driveSystem.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveSystem.tankDriveVolts(0, 0));
  }

/////////////////////////////////////////
  public void initTesting() {
    // visionSystem.disableLeds();
    visionSystem.enableLeds();
    visionSystem.setToVision();
    JoystickButton enableShooterButton = new JoystickButton(testController, kX.value);
    JoystickButton disableMagButton = new JoystickButton(testController, kY.value);
    JoystickButton shootButton = new JoystickButton(testController, kA.value);
    JoystickButton stopShooterButton = new JoystickButton(testController, kB.value);
    

    // turretLeftButton.whenPressed(shooterSystem.setTurretPower(0.5));

    enableShooterButton.whenPressed(() -> shooterSystem.setFlywheelPower(0.2), shooterSystem);
    disableMagButton.whenPressed(magSystem::disableMagazine, magSystem);

    shootButton.whenPressed(new RunShooter(shooterSystem));
    stopShooterButton.whenPressed(new StopShooter(shooterSystem));

    shooterSystem.setFlywheelPower(.2);

    

    
  }

  /** Shoots all the balls */ 
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