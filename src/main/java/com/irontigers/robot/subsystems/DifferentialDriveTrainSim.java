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


public class DifferentialDriveTrainSim extends SubsystemBase {


    private Encoder real_leftEncoder = new Encoder(0, 1);
    private Encoder real_rightEncoder = new Encoder(2, 3);
  
    private EncoderSim leftEncoder = new EncoderSim(real_leftEncoder);
    private EncoderSim rightEncoder = new EncoderSim(real_rightEncoder);
  
    private AnalogGyro real_gyro = new AnalogGyro(1);
  
    private AnalogGyroSim gyroSim = new AnalogGyroSim(real_gyro);


  /** Creates a new DifferentialDriveTrainSim. */
  public DifferentialDriveTrainSim(int numMotorEachSide, double gearingReduction, double massDriveTrain, double massRobot, double radiusWheel, double trackWidth, Matrix dDeviation) { // Don't know what data type is required

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
