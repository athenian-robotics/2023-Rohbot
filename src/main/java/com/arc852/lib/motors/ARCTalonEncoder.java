package com.arc852.lib.motors;

import com.arc852.Robot;
import lombok.Getter;
import reactor.core.publisher.Flux;
import reactor.core.publisher.Sinks;

public class ARCTalonEncoder extends ARCTalon {
  private final ARCBaseTalon motor;

  private final Sinks.Many<Double> positionSink;
  private final Sinks.Many<Double> velocitySink;
  @Getter private TalonState state = TalonState.ENCODER;

  protected ARCTalonEncoder(int id) {
    motor = new ARCBaseTalon(id);

    positionSink = Sinks.many().multicast().onBackpressureBuffer();
    velocitySink = Sinks.many().multicast().onBackpressureBuffer();

    Robot.getInstance()
        .addPeriodic(
            () -> {
              try {
                positionSink.tryEmitNext(getSelectedSensorPosition());
                velocitySink.tryEmitNext(getSelectedSensorVelocity());
              } catch (Exception e) {
                System.out.println(e.getMessage());
              }
            });
  }

  protected ARCTalonEncoder(ARCBaseTalon motor) {
    this.motor = motor;

    positionSink = Sinks.many().multicast().onBackpressureBuffer();
    velocitySink = Sinks.many().multicast().onBackpressureBuffer();

    Robot.getInstance()
        .addPeriodic(
            () -> {
              System.out.println("xx");
              positionSink.tryEmitNext(getSelectedSensorPosition());
              velocitySink.tryEmitNext(getSelectedSensorVelocity());
            });
  }

  public double getSelectedSensorPosition() {
    return motor.getSelectedSensorPosition();
  }

  public Flux<Double> getSelectedSensorPositionFlux() {
    return positionSink.asFlux().repeat();
  }

  public Flux<Double> getSelectedSensorVelocityFlux() {
    return velocitySink.asFlux().repeat();
  }

  public double getSelectedSensorVelocity() {
    return motor.getSelectedSensorVelocity();
  }

  @Override
  protected void setMotorType(TalonState state) {
    if (state == TalonState.MOTOR) {
      throw new IllegalArgumentException(
          "This class represent an encoder, either the underlying motor can be shared"
              + "an ARCTalonMotor or be an encoder");
    }
    this.state = state;
  }

  @Override
  protected ARCBaseTalon getMotor() {
    return motor;
  }
}
