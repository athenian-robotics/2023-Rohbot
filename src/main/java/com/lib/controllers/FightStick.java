package com.lib.controllers;

import edu.wpi.first.wpilibj.GenericHID;

public class FightStick extends GenericHID {

  public FightStick(int port) {
    super(port);
  }

  public boolean getButton(Button button) {
    return getRawButton(button.value);
  }

  public boolean getAButton() {
    return getRawButton(Button.A.value);
  }

  public boolean getBButton() {
    return getRawButton(Button.B.value);
  }

  public boolean getXButton() {
    return getRawButton(Button.X.value);
  }

  public boolean getYButton() {
    return getRawButton(Button.Y.value);
  }

  public boolean getLBButton() {
    return getRawButton(Button.LB.value);
  }

  public boolean getRBButton() {
    return getRawButton(Button.RB.value);
  }

  public boolean getL3Button() {
    return getRawButton(Button.L3.value);
  }

  public boolean getR3Button() {
    return getRawButton(Button.R3.value);
  }

  public enum Button {
    X(3), // Controller Fightstick - 3 | Fightstick - 1
    A(1), // Controller FightStick - 1 | Fightstick - 2
    B(2), // Controller Fightstick - 2 | Fightstick - 3
    Y(4),
    LB(5),
    RB(6),
    LT(7),
    RT(8),
    L3(11),
    R3(12);

    public final int value;

    Button(int value) {
      this.value = value;
    }
  }
}
