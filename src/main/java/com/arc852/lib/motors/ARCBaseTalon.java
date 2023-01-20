package com.arc852.lib.motors;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

// shamelessly stolen from somewhere
public class ARCBaseTalon extends WPI_TalonFX {
  protected double mLastSet = Double.NaN;
  protected TalonFXControlMode mLastControlMode = null;

  public ARCBaseTalon(int deviceNumber) {
    super(deviceNumber);
  }

  @Override
  public void set(TalonFXControlMode mode, double value) {
    if (value != mLastSet || mode != mLastControlMode) {
      mLastSet = value;
      mLastControlMode = mode;
      super.set(mode, value);
    }
  }
}
