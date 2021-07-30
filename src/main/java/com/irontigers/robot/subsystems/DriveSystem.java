/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.irontigers.robot.subsystems;

import com.irontigers.robot.Constants;
import com.irontigers.robot.Robot;
import com.irontigers.robot.Constants.Drive;
import com.irontigers.robot.sim.CANSparkMaxSim;
// import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
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

  // private AHRS navX;
  private ADXRS450_Gyro gyro;

  private CANEncoder leftEncoder;
  private CANEncoder rightEncoder;

  private DifferentialDriveOdometry odometer;
  private Pose2d robotPosition;
  private final Field2d gameField;

  private ShuffleboardTab tab;
  private NetworkTableEntry yOffset;

  private DifferentialDrivetrainSim driveSim;
  private CANSparkMaxSim frontLeftSim;
  private CANSparkMaxSim frontRightSim;
  private ADXRS450_GyroSim navXSim;

  private double leftMotorVolts;
  private double rightMotorVolts;

  /**
   * Creates a new DriveSystem.
   */
  public DriveSystem() {
    // tab = Shuffleboard.getTab("Tab 3");
    // yOffset = tab.add("Distance from left side", 13.46875 * 12).getEntry();

    frontLeft = new CANSparkMax(Drive.FRNT_LFT, CANSparkMaxLowLevel.MotorType.kBrushless);
    backLeft = new CANSparkMax(Drive.BCK_LFT, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftMotors = new SpeedControllerGroup(frontLeft, backLeft);

    leftEncoder = frontLeft.getEncoder();
    leftEncoder.setPositionConversionFactor(Drive.ENC_CNV_FCTR);
    leftEncoder.setVelocityConversionFactor(Drive.ENC_CNV_FCTR / 60);

    frontRight = new CANSparkMax(Drive.FRNT_RT, CANSparkMaxLowLevel.MotorType.kBrushless);
    backRight = new CANSparkMax(Drive.BCK_RT, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightMotors = new SpeedControllerGroup(frontRight, backRight);

    rightEncoder = frontRight.getEncoder();
    rightEncoder.setPositionConversionFactor(Drive.ENC_CNV_FCTR);
    rightEncoder.setVelocityConversionFactor(Drive.ENC_CNV_FCTR / 60);

    drive = new DifferentialDrive(leftMotors, rightMotors);

    // navX = new AHRS(); //RIP NavX
    gyro = new ADXRS450_Gyro(SPI.Port.kMXP);
    gyro.calibrate();

    odometer = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-gyro.getAngle()),
        new Pose2d(Units.feetToMeters(42.4375), Units.feetToMeters(13.46875), Rotation2d.fromDegrees(0)));
    
    gameField = new Field2d();

    if (!Robot.isReal()) {
      leftMotorVolts = 0;
      rightMotorVolts = 0;

      driveSim = new DifferentialDrivetrainSim(DCMotor.getNEO(2), 8.45,
          (12.5 / 2.2 * Math.pow(Units.inchesToMeters(10), 2))
              + (2.8 /* CIM motor */ * 2 / 2.2 + 2.0 /* Toughbox Mini- ish */)
                  * Math.pow(Units.inchesToMeters(26.0 / 2.0), 2),
          2.6605493765160295, Units.inchesToMeters(3), Units.inchesToMeters(26), null// VecBuilder.fill(0.001,
                                                                                                                     // 0.001,
                                                                                                                     // 0.001,
                                                                                                                     // 0.1,
                                                                                                                     // 0.1,
                                                                                                                     // 0.005,
                                                                                                                     // 0.005)
      );

      frontLeftSim = new CANSparkMaxSim(Drive.FRNT_LFT);
      frontRightSim = new CANSparkMaxSim(Drive.FRNT_RT);

      navXSim = new ADXRS450_GyroSim(gyro);

      register();
    }
  }

  private double getLeftDistance() {
    return leftEncoder.getPosition();
  }

  private double getRightDistance() {
    return -rightEncoder.getPosition();
  }

  private double getStartingYCoordinateFeet() {
    // return yOffset.getDouble(13.46875*12) / 12;
    return 13.46875;

  }
  public void initStartingPose() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    
    odometer.resetPosition(new Pose2d( Units.feetToMeters(42.4375), Units.feetToMeters(getStartingYCoordinateFeet()), Rotation2d.fromDegrees(-180)), Rotation2d.fromDegrees(-gyro.getAngle()));
  }

  public void drive(double ySpeed, double rotation) {
    drive.arcadeDrive(ySpeed, rotation, true);
  }

  public void setPose(Pose2d pose) {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    odometer.resetPosition(pose, gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    robotPosition = odometer.update(Rotation2d.fromDegrees(-gyro.getAngle()), getLeftDistance(), getRightDistance());

    SmartDashboard.putNumber("Left Encoder Distance m", getLeftDistance());
    SmartDashboard.putNumber("Right Encoder Distance m", getRightDistance());

    SmartDashboard.putNumber("Robot x Delta Pos", robotPosition.getTranslation().getX());
    SmartDashboard.putNumber("Robot Y Delta Pos", robotPosition.getTranslation().getY());
  }

  public Pose2d getRobotPosition() {
    return robotPosition;
  }

  @Override
  public void simulationPeriodic() {

    driveSim.setInputs(leftMotorVolts, rightMotorVolts);

    driveSim.update(0.02);

    // frontLeftSim.setPosition(driveSim.getLeftPositionMeters());
    // frontRightSim.setPosition(-driveSim.getRightPositionMeters());

    // frontLeftSim.setVelocity(driveSim.getLeftVelocityMetersPerSecond());
    // frontRightSim.setVelocity(-driveSim.getRightVelocityMetersPerSecond());

    navXSim.setAngle(driveSim.getHeading().getDegrees());
  }
}
