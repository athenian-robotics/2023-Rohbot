package com.arc852.subsystems;

import com.arc852.Constants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;

public class Elevator extends SubsystemBase {
  private final MotorControllerGroup elevatorMotors;
  private final GenericEntry position;
  private final GenericEntry velocity;
  private final LinearSystemLoop<N2, N1, N1> loop;
  private final WPI_TalonFX leftMotor;
  private final WPI_TalonFX rightMotor;
  private double pos = 0;
  private final double TICKS_TO_ROHUNIT =
      1.0 / 20_000; // fake unit, around 10 rotatoins or 20k ticks

  public Elevator() {
    leftMotor = new WPI_TalonFX(Constants.ElevatorConstants.LEFT_MOTOR);
    rightMotor = new WPI_TalonFX(Constants.ElevatorConstants.RIGHT_MOTOR);
    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);
    this.elevatorMotors = new MotorControllerGroup(leftMotor, rightMotor);
    ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
    position = tab.add("Elevator Position", leftMotor.getSelectedSensorPosition()).getEntry();
    velocity = tab.add("Elevator Speed", leftMotor.getSelectedSensorVelocity()).getEntry();

    var sys =
        LinearSystemId.identifyPositionSystem(
            Constants.ElevatorConstants.kV, Constants.ElevatorConstants.kA);

    KalmanFilter<N2, N1, N1> filter =
        new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            sys,
            VecBuilder.fill(0.1, 0.1),
            VecBuilder.fill(TICKS_TO_ROHUNIT),
            0.02); // use rmse for velo and accel
    LinearQuadraticRegulator<N2, N1, N1> controller =
        new LinearQuadraticRegulator<>(
            sys,
            VecBuilder.fill(0.1, 0.1),
            VecBuilder.fill(12),
            0.02); // error tolerance for pos, velo and then control effort
    loop = new LinearSystemLoop<>(sys, controller, filter, 12, 0.02);
  }

  /**
   * Sets the elevator to a certain percent of max height
   *
   * @param percent 0-1
   * @return Command
   */
  public Command set(double percent) {
    return new InstantCommand(
        () ->
            pos =
                percent
                        * (Constants.ElevatorConstants.MAX_HEIGHT
                            - Constants.ElevatorConstants.MIN_HEIGHT)
                    + Constants.ElevatorConstants.MIN_HEIGHT,
        this);
  }

  @Override
  public void periodic() {
    loop.setNextR(pos * TICKS_TO_ROHUNIT, 0);
    loop.correct(VecBuilder.fill(leftMotor.getSelectedSensorPosition()));
    loop.predict(0.02);
    elevatorMotors.setVoltage(
        loop.getU(0)
            + Constants.ElevatorConstants.kS * loop.getNextR(1)
            + Constants.ElevatorConstants.kG);

    position.setDouble(leftMotor.getSelectedSensorPosition());
    velocity.setDouble(leftMotor.getSelectedSensorVelocity());
  }
}
