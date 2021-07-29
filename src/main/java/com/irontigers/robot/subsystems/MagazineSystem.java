/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.irontigers.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.irontigers.robot.Dashboard;
import com.irontigers.robot.Constants.Magazine;
import com.irontigers.robot.Dashboard.TAB;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MagazineSystem extends SubsystemBase {
  private WPI_TalonFX magazineMotor;
  private WPI_VictorSPX intakeMotor;

  private boolean intakeEnabled = false;
  private boolean magazineEnabled = false;

  private int storedBalls;

  private final AnalogInput bottomBallSensor;
  private final AnalogInput topBallSensor;

  private Solenoid ballGate;

  private MedianFilter botttomBallFilter;

  private boolean ballGateOpen = false;
  private boolean ballGateInitComplete = false;

  private Dashboard dash;

  /**
   * Creates a new MagazineSystem.
   */
  public MagazineSystem() {
    dash = Dashboard.getInstance();
    magazineMotor = new WPI_TalonFX(Magazine.MAG_ADDR);
    magazineMotor.setInverted(true);
    // enableMagazine();

    intakeMotor = new WPI_VictorSPX(Magazine.INTAKE_ADDR);

    storedBalls = 3; // 3;

    bottomBallSensor = new AnalogInput(Magazine.BOT_SENSOR_PORT);
    topBallSensor = new AnalogInput(Magazine.TOP_SENSOR_PORT);

    ballGate = new Solenoid(Magazine.GATE_PORT);
    closeGate();

    botttomBallFilter = new MedianFilter(20);

    dash.getTab(TAB.AUTO).addNumber("Balls", this::getStoredBalls).withWidget(BuiltInWidgets.kDial).withPosition(0, 0)
        .withSize(4, 3).withProperties(Map.of("min", 0, "max", 3));

    dash.getTab(TAB.TELEOP).addNumber("Balls", this::getStoredBalls).withWidget(BuiltInWidgets.kDial).withPosition(0, 0)
        .withSize(4, 3).withProperties(Map.of("min", 0, "max", 3));

    dash.getTab(TAB.TELEOP).addBoolean("Intake", this::isIntakeEnabled).withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0, 3).withSize(1, 1);

    dash.getTab(TAB.TELEOP).addBoolean("Magazine", this::isIntakeEnabled).withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(1, 3).withSize(1, 1);
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
    SmartDashboard.putNumber("Filtered sensor reading", botttomBallFilter.calculate(bottomBallSensor.getAverageValue()));
    SmartDashboard.putNumber("Bottom sensor reading", bottomBallSensor.getAverageValue());
    SmartDashboard.putNumber("Top sensor reading", topBallSensor.getAverageValue());
    SmartDashboard.putBoolean("Ball gate open", ballGateOpen);
  }

  public int getStoredBalls() {
    return storedBalls;
  }

  public void incrementBalls() {
    if (storedBalls < 3) {
      storedBalls++;
    }
  }

  public void decrementBalls() {
    closeGate();
    if (storedBalls > 0) {
      storedBalls--;
    }
  }

  public void setStoredBalls(int balls) {
    if (balls > 3 || balls < 0) {
      throw new IllegalArgumentException("Stored balls must be at least 0 and no more than 3.");
    } else {
      storedBalls = balls;
    }
  }

  public AnalogInput getBottomBallSensor() {
    return bottomBallSensor;
  }

  public boolean isMagFull() { return storedBalls >= 3; }

  public AnalogInput getTopBallSensor() {
    return topBallSensor;
  }

  public void openGate() {
    ballGate.set(false);
    ballGateOpen = true;
  }

  public void closeGate() {
    ballGate.set(true);
    ballGateOpen = false;
  }
}
