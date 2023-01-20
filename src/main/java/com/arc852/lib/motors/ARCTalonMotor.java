package com.arc852.lib.motors;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import lombok.Getter;

public class ARCTalonMotor extends ARCTalon {
  private final ARCBaseTalon motor;
  @Getter private TalonState state = TalonState.MOTOR;

  protected ARCTalonMotor(int id) {
    motor = new ARCBaseTalon(id);
  }

  public ARCTalonMotor(ARCBaseTalon motor) {
    this.motor = motor;
  }

  public void set(double value) {
    motor.set(value);
  }

  @Override
  protected void setMotorType(TalonState state) {
    if (state == TalonState.ENCODER) {
      throw new IllegalArgumentException("trying to trick a motor into think it's an encoder");
    }
    this.state = state;
  }

  @Override
  protected ARCBaseTalon getMotor() {
    return motor;
  }

  public void configAllSettings(TalonFXConfiguration swerveAngleFXConfig) {
    motor.configAllSettings(swerveAngleFXConfig);
  }
}
