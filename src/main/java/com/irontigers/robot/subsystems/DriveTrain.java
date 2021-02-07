// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package com.irontigers.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpiutil.math.VecBuilder;
// import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
// import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
// import edu.wpi.first.wpilibj.simulation.EncoderSim;
// import edu.wpi.first.wpilibj.system.plant.DCMotor;
// import edu.wpi.first.wpilibj.util.Units;

// import com.ctre.phoenix.motorcontrol.can.TalonFX;

// import edu.wpi.first.wpilibj.AnalogGyro;
// import edu.wpi.first.wpilibj.Encoder;

// import com.irontigers.robot.Constants;



// public class DriveTrain extends SubsystemBase {
  
//   private Encoder real_leftEncoder = new Encoder(0, 1);
//   private Encoder real_rightEncoder = new Encoder(2, 3);
  
//   private EncoderSim leftEncoder = new EncoderSim(real_leftEncoder);
//   private EncoderSim rightEncoder = new EncoderSim(real_rightEncoder);
  
//   private AnalogGyro real_gyro = new AnalogGyro(1);
  
//   private AnalogGyroSim gyroSim = new AnalogGyroSim(real_gyro);

//   DifferentialDrivetrainSim myDriveSim = new DifferentialDrivetrainSim(
//     DCMotor.getFalcon500(1),
//     7.29, 
//     7.5, 
//     110.0,
//     Units.inchesToMeters(3),
//     0.7112,
//     null
//   );



//   /** Creates a new DriveTrain. */
//   public DriveTrain() {
//     // real_leftEncoder.setDistancePerPulse((2 * Math.PI * Units.inchesToMeters(3) / ));
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

//   @Override
//   public void simulationPeriodic() {
//     // This method will be called once per scheduler run during simulation
//   }
// }
