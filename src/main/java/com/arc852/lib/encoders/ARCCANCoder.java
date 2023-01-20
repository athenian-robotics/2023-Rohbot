package com.arc852.lib.encoders;

import com.arc852.Robot;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import reactor.core.publisher.Flux;
import reactor.core.publisher.Sinks;

public class ARCCANCoder {
  private final CANCoder encoder;
  private Sinks.Many<Double> posSink;

  public ARCCANCoder(int id) {
    encoder = new CANCoder(id);
    posSink = Sinks.many().multicast().onBackpressureBuffer();
    Robot.getInstance().addPeriodic(() -> posSink.tryEmitNext(encoder.getAbsolutePosition()));
  }

  public Flux<Double> getPosition() {
    return posSink.asFlux().repeat();
  }

  public void configFactoryDefault() {
    encoder.configFactoryDefault();
  }

  public void configAllSettings(CANCoderConfiguration swerveCanCoderConfig) {
    encoder.configAllSettings(swerveCanCoderConfig);
  }

  public void setPositionToAbsolute() {
    encoder.setPositionToAbsolute();
  }
}
