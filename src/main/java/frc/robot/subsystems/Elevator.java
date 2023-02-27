package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private final MotorControllerGroup elevatorMotors;
  private final GenericEntry position;
  private final GenericEntry velocity;
  private final LinearSystemLoop<N2, N1, N1> loop;
  private final WPI_TalonFX leftMotor;
  private final WPI_TalonFX rightMotor;
  private double pos = 0;
  private static State state = State.VELO;

  public Elevator() {
    leftMotor = new WPI_TalonFX(Constants.ElevatorConstants.LEFT_MOTOR);
    rightMotor = new WPI_TalonFX(Constants.ElevatorConstants.RIGHT_MOTOR);
    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);
    this.elevatorMotors = new MotorControllerGroup(leftMotor, rightMotor);
    ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
    position = tab.add("Elevator Position", leftMotor.getSelectedSensorPosition()).getEntry();
    velocity = tab.add("Elevator Speed", leftMotor.getSelectedSensorVelocity()).getEntry();

    LinearSystem<N2, N1, N1> sys =
        LinearSystemId.createElevatorSystem(DCMotor.getFalcon500(2), 0.1, 0.1, 0.1);
    //        sys = LinearSystemId.identifyPositionSystem(Constants.Elevator.kv,
    // Constants.Elevator.ka)
    KalmanFilter<N2, N1, N1> filter =
        new KalmanFilter<>(
            Nat.N2(), Nat.N1(), sys, VecBuilder.fill(0.1, 0.1), VecBuilder.fill(0.1), 0.02);
    LinearQuadraticRegulator<N2, N1, N1> controller =
        new LinearQuadraticRegulator<N2, N1, N1>(
            sys, VecBuilder.fill(0.1, 0.1), VecBuilder.fill(0.1), 0.02);
    loop = new LinearSystemLoop<>(sys, controller, filter, 12, 0.02);
  }

  enum State {
    VELO,
    POS
  }

  /**
  * Moves the elevator up.
  * 
  * @return Command
  */
  public Command moveUp() {
    return new StartEndCommand(
        () -> {
          state = State.VELO;
          if (leftMotor.getSelectedSensorPosition() > Constants.ElevatorConstants.MAX_HEIGHT) {
            elevatorMotors.set(-Constants.ElevatorConstants.ELEVATOR_MOVE_SPEED);
          } else {
            elevatorMotors.set(0);
            System.out.println("Stopped");
          }
        },
        () -> elevatorMotors.set(0),
        this);
  }

  /**
   * Moves the elevator down.
   * 
   * @return Command
   */
  public Command moveDown() {
    return new StartEndCommand(
        () -> {
          state = State.VELO;
          if (leftMotor.getSelectedSensorPosition() < Constants.ElevatorConstants.MIN_HEIGHT) {
            elevatorMotors.set(Constants.ElevatorConstants.ELEVATOR_MOVE_SPEED);
          } else {
            elevatorMotors.set(0);
            System.out.println("Stopped");
          }
        },
        () -> elevatorMotors.set(0),
        this);
  }

  /**
   * Sets the elevator to a certain percent of max height
   *
   * @param percent 0-1
   * @return Command 
   */
  public Command set(double percent) {
    return new RunCommand(
            () -> {
              state = State.POS;
              pos =
                  percent
                          * (Constants.ElevatorConstants.MAX_HEIGHT
                              - Constants.ElevatorConstants.MIN_HEIGHT)
                      + Constants.ElevatorConstants.MIN_HEIGHT;
            },
            this)
        .until(() -> loop.getError().get(0, 0) < 0.1 && loop.getError().get(1, 0) < 0.1);
  }

  @Override
  public void periodic() {
    if (state == State.POS) {
      loop.setNextR(pos, 0);
      loop.correct(VecBuilder.fill(leftMotor.getSelectedSensorPosition()));
      loop.predict(0.02);
      elevatorMotors.setVoltage(loop.getU(0) + Constants.ElevatorConstants.kS);
    }

    position.setDouble(leftMotor.getSelectedSensorPosition());
    velocity.setDouble(leftMotor.getSelectedSensorVelocity());
  }
}
