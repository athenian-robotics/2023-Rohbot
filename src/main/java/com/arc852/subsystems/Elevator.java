package com.arc852.subsystems;

import com.arc852.Constants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.function.DoubleSupplier;

import static com.arc852.Constants.Elevator.*;
import static com.arc852.Constants.Elevator.MAX_HEIGHT;
import static com.arc852.Constants.Elevator.TICKS_TO_METERS;

public class Elevator extends SubsystemBase implements Loggable {
  private final MotorControllerGroup elevatorMotors;
  private final GenericEntry position;
  private final GenericEntry velocity;
  private final LinearSystemLoop<N2, N1, N1> loop;
  private final WPI_TalonFX leftMotor;
  private final WPI_TalonFX rightMotor;
  @Log private double pos = 0;
  SlewRateLimiter limiter = new SlewRateLimiter(2.8); // m/s

  public Elevator() {
    leftMotor = new WPI_TalonFX(LEFT_MOTOR);
    rightMotor = new WPI_TalonFX(RIGHT_MOTOR);
    leftMotor.setInverted(false);
    rightMotor.setInverted(false);
    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);
    this.elevatorMotors = new MotorControllerGroup(leftMotor, rightMotor);
    ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
    position = tab.add("Elevator Position", leftMotor.getSelectedSensorPosition()).getEntry();
    velocity = tab.add("Elevator Speed", leftMotor.getSelectedSensorVelocity()).getEntry();

    var sys = LinearSystemId.identifyPositionSystem(kV, kA);

    KalmanFilter<N2, N1, N1> filter =
        new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            sys,
            VecBuilder.fill(0.061, 0.061),
            VecBuilder.fill(TICKS_TO_METERS),
            0.02); // use rmse for velo and accel
    LinearQuadraticRegulator<N2, N1, N1> controller =
        new LinearQuadraticRegulator<>(
            sys,
            VecBuilder.fill(0.1, 0.1),
            VecBuilder.fill(8),
            0.02); // error tolerance for pos, velo and then control effort

    loop = new LinearSystemLoop<>(sys, controller, filter, 12, 0.02);
  }

  /**
   * Sets the elevator to a certain percent of max height
   *
   * @param percent 0-1
   * @return Command
   */
  public Command set(DoubleSupplier percent) {
    return new RunCommand(
        () ->
            pos =
                percent.getAsDouble()
                        * (MAX_HEIGHT - MIN_HEIGHT)
                    + MIN_HEIGHT,
        this);
  }

  @Log
  public boolean atSetpoint() {
    return Math.abs(pos - leftMotor.getSelectedSensorPosition() * TICKS_TO_METERS) < 0.15;
  }

  @Log
  public double positionPercent() {
    return leftMotor.getSelectedSensorPosition() * TICKS_TO_METERS / MAX_HEIGHT;
  }

  @Override
  public void periodic() {
    loop.setNextR(limiter.calculate(pos), 0);
    loop.correct(
        VecBuilder.fill(
            leftMotor.getSelectedSensorPosition() * TICKS_TO_METERS));
    loop.predict(0.02);
    elevatorMotors.setVoltage(
        loop.getU(0) + kS * loop.getNextR(1) + kG);

    position.setDouble(leftMotor.getSelectedSensorPosition());
    velocity.setDouble(leftMotor.getSelectedSensorVelocity());
  }
}
