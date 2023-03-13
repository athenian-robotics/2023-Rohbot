package com.arc852.subsystems;

import com.arc852.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.*;

public class Arm extends SubsystemBase {
  private final LinearSystemLoop<N2, N1, N1> loop;
  private double pos = 0;
  private final WPI_TalonFX leftMotor;
  private final WPI_TalonFX rightMotor;
  private final MotorControllerGroup armGroup;
  private final double TICKS_TO_RAD = 2 * Math.PI / 2048; // get gear rato from ********** cad team

  public Arm() {
    leftMotor = new WPI_TalonFX(Constants.Arm.LEFT_MOTOR);
    rightMotor = new WPI_TalonFX(Constants.Arm.RIGHT_MOTOR);
    armGroup = new MotorControllerGroup(leftMotor, rightMotor);

    var sys =
        LinearSystemId.identifyPositionSystem(Constants.Arm.kV, Constants.Arm.kA);

    KalmanFilter<N2, N1, N1> filter =
        new KalmanFilter<>(
            Nat.N2(), Nat.N1(), sys, VecBuilder.fill(0.1, 0.1), VecBuilder.fill(TICKS_TO_RAD), 0.02);
    LinearQuadraticRegulator<N2, N1, N1> controller =
        new LinearQuadraticRegulator<>(sys, VecBuilder.fill(0.1, 0.1), VecBuilder.fill(0.1), 0.02);
    loop = new LinearSystemLoop<>(sys, controller, filter, 12, 0.02);
  }

  /**
   * Sets the arm to
   *
   * @param radians angle to set the arm to
   * @return command that does the thing
   */
  public Command set(double radians) {
    return new InstantCommand(
        () ->
            pos =
                MathUtil.clamp(
                    radians, Constants.Arm.MIN_ANGLE, Constants.Arm.MAX_ANGLE),
        this);
  }

  @Override
  public void periodic() {
    loop.setNextR(pos, 0);
    loop.correct(VecBuilder.fill(leftMotor.getSelectedSensorPosition() * TICKS_TO_RAD));
    loop.predict(0.02);
    armGroup.setVoltage(
        loop.getU(0)
            + Constants.Arm.kS * Math.signum(loop.getNextR(1))
            + Constants.Arm.kG * Math.cos(loop.getNextR(0)));
  }
}
