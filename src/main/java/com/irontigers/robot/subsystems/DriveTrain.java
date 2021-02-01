// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;

// import frc.robot.Constants.Encoders;

public class DriveTrain extends SubsystemBase {

  DifferentialDriveTrainSim myDriveSim = new DifferentialDriveTrainSim(
    DCMotor.getFalcon500(1),
    7.29, 
    7.5, 
    110.0,
    Units.inchesToMeters(3),
    0.7112,
    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)  // Line with error
  );
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    // real_leftEncoder.setDistancePerPulse((2 * Math.PI * Units.inchesToMeters(3) / Encoder.get));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
