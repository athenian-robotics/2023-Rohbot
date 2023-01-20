package com.arc852.lib.motors;

public abstract class ARCTalon {
  protected abstract TalonState getState();

  protected abstract void setMotorType(TalonState state);

  protected abstract ARCBaseTalon getMotor();
}
