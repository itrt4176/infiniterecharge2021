package com.irontigers.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;

public class CorrectXboxController extends XboxController {
    /**
   * Construct an instance of a joystick. The joystick index is the USB port on the drivers
   * station.
   *
   * @param port The port on the Driver Station that the joystick is plugged into.
   */
  public CorrectXboxController(final int port) {
    super(port);
  }

  @Override
  public double getY(Hand hand) {
      return -super.getY(hand);
  }
}
