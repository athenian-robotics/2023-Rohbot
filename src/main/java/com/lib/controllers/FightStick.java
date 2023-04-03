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

  public boolean getShareButton() {
    return getRawButton(Button.SHARE.value);
  }

  public boolean getOptionsButton() {
    return getRawButton(Button.OPTIONS.value);
  }

  public boolean getL3Button() {
    return getRawButton(Button.L3.value);
  }

  public boolean getR3Button() {
    return getRawButton(Button.R3.value);
  }

  public enum Button {
    A(1),
    B(2),
    X(3),
    Y(4),
    LB(5),
    RB(6),
    SHARE(7),
    OPTIONS(8),
    L3(9),
    R3(10);

    public final int value;

    Button(int value) {
      this.value = value;
    }
  }
}
