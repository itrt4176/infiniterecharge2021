/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.irontigers.robot.subsystems;

import com.irontigers.robot.Constants.Drive;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.Supplier;

import com.irontigers.robot.Constants;
import com.irontigers.robot.Robot;
import com.irontigers.robot.sim.CANSparkMaxSim;
import com.irontigers.robot.sim.DifferentialDriveCompat;
import com.irontigers.robot.sim.NavXSim;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSystem extends SubsystemBase {
  private CANSparkMax frontLeft;
  private CANSparkMax backLeft;
  private CANSparkMax frontRight;
  private CANSparkMax backRight;

  private SpeedControllerGroup leftMotors;
  private SpeedControllerGroup rightMotors;

  private DifferentialDrive drive;

  private AHRS navX;

  private CANEncoder leftEncoder;
  private CANEncoder rightEncoder;

  private DifferentialDriveOdometry odometer;
  private Pose2d robotPosition;
  private final Field2d gameField;

  private ShuffleboardTab tab;
  private NetworkTableEntry yOffset;

  // Simulation objects
  private DifferentialDrivetrainSim driveSim;
  private CANSparkMaxSim frontLeftSim;
  private CANSparkMaxSim frontRightSim;
  private NavXSim navXSim;

  private final String positionKey = "Position";
  private final String velocityKey = "Velocity";


  /**
   * Creates a new DriveSystem.
   */
  public DriveSystem() {
    tab = Shuffleboard.getTab("Tab 3");
    yOffset = tab.add("Distance from left side", 13.46875*12).getEntry();

    frontLeft = new CANSparkMax(Drive.FRNT_LFT, CANSparkMaxLowLevel.MotorType.kBrushless);
    backLeft = new CANSparkMax(Drive.BCK_LFT, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftMotors = new SpeedControllerGroup(frontLeft, backLeft);

    leftEncoder = frontLeft.getEncoder();
    leftEncoder.setPositionConversionFactor(Drive.ENC_CNV_FCTR);
    leftEncoder.setVelocityConversionFactor(Drive.ENC_CNV_FCTR / 60);

    frontRight = new CANSparkMax(Drive.FRNT_RT, CANSparkMaxLowLevel.MotorType.kBrushless);
    // frontRight.setInverted(true);
    backRight = new CANSparkMax(Drive.BCK_RT, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightMotors = new SpeedControllerGroup(frontRight, backRight);

    rightEncoder = frontRight.getEncoder();
    rightEncoder.setPositionConversionFactor(Drive.ENC_CNV_FCTR);
    rightEncoder.setVelocityConversionFactor(Drive.ENC_CNV_FCTR / 60);

    // drive = new DifferentialDriveCompat(leftMotors, rightMotors);
    drive = new DifferentialDrive(leftMotors, rightMotors);

    navX = new AHRS();

    odometer = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-navX.getAngle()),
        new Pose2d(Units.feetToMeters(42.4375), Units.feetToMeters(13.46875), Rotation2d.fromDegrees(0)));
    
    gameField = new Field2d();


    if (!Robot.isReal()) {
      driveSim = new DifferentialDrivetrainSim(
        DCMotor.getNEO(2),
        8.45, 
        3, 
        49.8952,
        Units.inchesToMeters(3),
        Units.inchesToMeters(26),
        null//VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
      );

      frontLeftSim = new CANSparkMaxSim(Drive.FRNT_LFT);
      frontRightSim = new CANSparkMaxSim(Drive.FRNT_RT);

      navXSim = new NavXSim();

      register();
    }
  }

  public Supplier<Pose2d> getPose2d() {
    return () -> odometer.getPoseMeters(); 
  }

  public Supplier<DifferentialDriveWheelSpeeds> getWheelSpeeds() {
    return () -> new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), -rightEncoder.getVelocity());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometer.resetPosition(pose, navX.getRotation2d());
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  private double getLeftDistance() {
    return leftEncoder.getPosition();
  }

  private double getRightDistance() {
    return -rightEncoder.getPosition();
  }

  private double getStartingYCoordinateFeet() {
    return yOffset.getDouble(13.46875*12) / 12;

  }
  public void initStartingPose() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    
    odometer.resetPosition(new Pose2d(Units.feetToMeters(42.4375), Units.feetToMeters(getStartingYCoordinateFeet()),
        Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(-navX.getAngle()));
  }

  public void drive(double ySpeed, double rotation) {
    drive.arcadeDrive(ySpeed, rotation, true);
  }

  // @FunctionalInterface
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(-rightVolts);
    drive.feed();
  }

  public static Trajectory path(String pathName) {
    String jsonFile = "paths/output/" + pathName + ".wpilib.json";
    Trajectory trajectory = new Trajectory();

    try {
      Path toTrajectory = Filesystem.getDeployDirectory().toPath().resolve(jsonFile);
      trajectory = TrajectoryUtil.fromPathweaverJson(toTrajectory);
      return trajectory;
    } catch (IOException e) {
      DriverStation.reportError("Unable to open trajectory" + jsonFile, e.getStackTrace());
      return null;
    }
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    robotPosition = odometer.update(Rotation2d.fromDegrees(-navX.getAngle()), getLeftDistance(), getRightDistance());
    gameField.setRobotPose(robotPosition);

    SmartDashboard.putNumber("Left Encoder Distance m", getLeftDistance());
    SmartDashboard.putNumber("Right Encoder Distance m", getRightDistance());

    SmartDashboard.putNumber("Robot x Delta Pos", robotPosition.getTranslation().getX());
    SmartDashboard.putNumber("Robot Y Delta Pos", robotPosition.getTranslation().getY());

    SmartDashboard.putData(gameField);
    NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
    NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");

    var translation = odometer.getPoseMeters().getTranslation();
    m_xEntry.setNumber(translation.getX());
    m_yEntry.setNumber(translation.getY());
  }

  public Pose2d getRobotPosition() {
    return robotPosition;
  }



  // @Override
  // public void simulationPeriodic() {

  //   driveSim.setInputs(drive.getLeftMotorOutput() * RobotController.getInputVoltage(),
  //       -drive.getRightMotorOutput() * RobotController.getInputVoltage());

  //   driveSim.update(0.02);                     

  //   frontLeftSim.setPosition(driveSim.getLeftPositionMeters());
  //   frontRightSim.setPosition(-driveSim.getRightPositionMeters());

  //   frontLeftSim.setVelocity(driveSim.getLeftVelocityMetersPerSecond());
  //   frontRightSim.setVelocity(-driveSim.getRightVelocityMetersPerSecond());

  //   navXSim.setAngle(driveSim.getHeading().getDegrees());
  // }
}