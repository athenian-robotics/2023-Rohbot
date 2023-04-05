package com.lib.controllers;

import edu.wpi.first.wpilibj.Joystick;

public class Thrustmaster extends Joystick {
  /**
   * Construct an instance of a device.
   *
   * @param port The port index on the Driver Station that the device is plugged into.
   */
  public Thrustmaster(int port) {
    super(port);
  }

  public enum Button {
    TRIGGER(1),
    BOTTOM(2),
    LEFT(3),
    RIGHT(4),
    LEFT_OUTSIDE_TOP(5),
    LEFT_MIDDLE_TOP(6),
    LEFT_INSIDE_TOP(7),
    LEFT_INSIDE_BOTTOM(8),
    LEFT_MIDDLE_BOTTOM(9),
    LEFT_OUTSIDE_BOTTOM(10),
    RIGHT_OUTSIDE_TOP(11),
    RIGHT_MIDDLE_TOP(12),
    RIGHT_INSIDE_TOP(13),
    RIGHT_INSIDE_BOTTOM(14),
    RIGHT_MIDDLE_BOTTOM(15),
    RIGHT_OUTSIDE_BOTTOM(16);

    public final int val;

    Button(int i) {
      val = i;
    }
  }
}
