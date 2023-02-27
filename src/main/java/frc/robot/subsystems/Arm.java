package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private final WPI_TalonFX armRotate;
  private final LinearSystemLoop<N2, N1, N1> loop;
  private double pos = 0;
  private static Arm.State state = Arm.State.POS;

  public Arm() {
    armRotate = new WPI_TalonFX(Constants.ArmConstants.armMotorID);

    LinearSystem<N2, N1, N1> sys =
        LinearSystemId.createSingleJointedArmSystem(DCMotor.getFalcon500(1), 0.1, 0.1);
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
   * Moves the arm up
   * @return command that does the thing
   */
  public Command moveUp() {
    return new StartEndCommand(
        () -> {
          state = State.VELO;
          if (armRotate.getSelectedSensorPosition() > Constants.ArmConstants.MAX_HEIGHT) {
            armRotate.set(-Constants.ArmConstants.ARM_MOVE_SPEED);
          } else {
            armRotate.set(0);
            System.out.println("Arm Stopped");
          }
        },
        () -> armRotate.set(0));
  }

  /**
   * Moves the arm down
   * @return command that does the thing
   */
  public Command moveDown() {
    return new StartEndCommand(
        () -> {
          state = State.VELO;
          if (armRotate.getSelectedSensorPosition() < Constants.ArmConstants.MIN_HEIGHT) {
            armRotate.set(Constants.ArmConstants.ARM_MOVE_SPEED);
          } else {
            armRotate.set(0);
            System.out.println("Arm Stopped");
          }
        },
        () -> armRotate.set(0));
  }

  /**
   * Sets the arm to a certain percent of max height
   *
   * @param percent 0-1
   * @return command that does the thing
   */
  public Command set(double percent) {
    return new RunCommand(
            () -> {
              state = Arm.State.POS;
              pos =
                  percent * (Constants.ArmConstants.MAX_HEIGHT - Constants.ArmConstants.MIN_HEIGHT)
                      + Constants.ArmConstants.MIN_HEIGHT;
            },
            this)
        .until(() -> loop.getError().get(0, 0) < 0.1 && loop.getError().get(1, 0) < 0.1);
  }

  @Override
  public void periodic() {
    if (state == Arm.State.POS) {
      loop.setNextR(pos, 0);
      loop.correct(VecBuilder.fill(armRotate.getSelectedSensorPosition()));
      loop.predict(0.02);
      armRotate.setVoltage(loop.getU(0) + Constants.ArmConstants.kS);
    }
  }
}
