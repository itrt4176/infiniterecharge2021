/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.irontigers.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.irontigers.robot.Constants.Magazine;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MagazineSystem extends SubsystemBase {
  private WPI_TalonFX magazineMotor;
  private WPI_VictorSPX intakeMotor;

  private boolean intakeEnabled = false;
  private boolean magazineEnabled = false;

  private int storedBalls;

  // private final AnalogInput bottomBallSensor;
  // private final AnalogInput topBallSensor;

  private Solenoid ballGate;

  private MedianFilter botttomBallFilter;

  private boolean ballGateOpen = false;
  private boolean ballGateInitComplete = false;

  /**
   * Creates a new MagazineSystem.
   */
  public MagazineSystem() {
    magazineMotor = new WPI_TalonFX(Magazine.MAG_ADDR);
    magazineMotor.setInverted(true);
    // enableMagazine();

    intakeMotor = new WPI_VictorSPX(Magazine.INTAKE_ADDR);

    storedBalls = 3; // 3;

    // bottomBallSensor = new AnalogInput(Magazine.BOT_SENSOR_PORT);
    // topBallSensor = new AnalogInput(Magazine.TOP_SENSOR_PORT);

    ballGate = new Solenoid(Magazine.GATE_PORT);
    closeGate();

    botttomBallFilter = new MedianFilter(20);
  }

  public void enableMagazine() {
    magazineMotor.set(Magazine.MAG_SPD);
    magazineEnabled = true;
  }

  public void disableMagazine() {
    magazineMotor.set(0);
    magazineEnabled = false;
  }

  public void enableIntake() {
    intakeMotor.set(Magazine.INTAKE_SPD);
    intakeEnabled = true;
  }

  public void disableIntake() {
    intakeMotor.set(0);
    intakeEnabled = false;
  }

  public boolean isIntakeEnabled() {
    return intakeEnabled;
  }

  public boolean isMagazineEnabled() {
    return magazineEnabled;
  }

  public void stop() {
    magazineMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Ball count", storedBalls);
    // SmartDashboard.putNumber("Filtered sensor reading", botttomBallFilter.calculate(bottomBallSensor.getAverageValue()));
    // SmartDashboard.putNumber("Bottom sensor reading", bottomBallSensor.getAverageValue());
    // SmartDashboard.putNumber("Top sensor reading", topBallSensor.getAverageValue());
    SmartDashboard.putBoolean("Ball gate open", ballGateOpen);
  }

  public int getStoredBalls() {
    return storedBalls;
  }

  public void incrementBalls() {
    storedBalls++;
  }

  public void decrementBalls() {
    closeGate();
    storedBalls--;
  }

  // public AnalogInput getBottomBallSensor() {
  //   return bottomBallSensor;
  // }

  public boolean isMagFull() { return storedBalls >= 3; }

  // public AnalogInput getTopBallSensor() {
  //   return topBallSensor;
  // }

  public void openGate() {
    ballGate.set(false);
    ballGateOpen = true;
  }

  public void closeGate() {
    ballGate.set(true);
    ballGateOpen = false;
  }
}
