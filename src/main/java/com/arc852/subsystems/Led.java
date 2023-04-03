package com.arc852.subsystems;

import com.ctre.phoenix.CANifier;

public class Led {
  private final CANifier canifier;

  public Led(int canifierId) {
    canifier = new CANifier(canifierId);
  }

  public void on() {
    canifier.setLEDOutput(1, CANifier.LEDChannel.LEDChannelA);
  }

  public void off() {
    canifier.setLEDOutput(0, CANifier.LEDChannel.LEDChannelA);
  }
}
