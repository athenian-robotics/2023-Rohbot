package com.arc852.subsystems;

import com.arc852.Constants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
import edu.wpi.first.wpilibj2.command.*;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Arm extends SubsystemBase implements Loggable {
  private final LinearSystemLoop<N2, N1, N1> loop;
  private double pos = 0;
  private final WPI_TalonFX bottomMotor;
  private final double TICKS_TO_RAD =
      2
          * Math.PI
          / 2048
          / Constants.Arm.gearRatio; // 1:5 where 1 is the motor and 5 is the arm gear

  public Arm() {
    bottomMotor = new WPI_TalonFX(Constants.Arm.LEFT_MOTOR);
    bottomMotor.setInverted(true);
    bottomMotor.setNeutralMode(NeutralMode.Brake);

    var sys = LinearSystemId.identifyPositionSystem(Constants.Arm.kV, Constants.Arm.kA);

    KalmanFilter<N2, N1, N1> filter =
        new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            sys,
            VecBuilder.fill(0.02, 0.02),
            VecBuilder.fill(TICKS_TO_RAD),
            0.02);
    LinearQuadraticRegulator<N2, N1, N1> controller =
        new LinearQuadraticRegulator<>(
            sys, VecBuilder.fill(0.0015, 0.02), VecBuilder.fill(12), 0.02);
    loop = new LinearSystemLoop<>(sys, controller, filter, 12, 0.02);
  }

  /**
   * Sets the arm to
   *
   * @param radians angle to set the arm to
   * @return command that does the thing
   */
  public Command set(double radians) {
    return runOnce(
        () -> pos = MathUtil.clamp(radians, Constants.Arm.MIN_ANGLE, Constants.Arm.MAX_ANGLE));
  }

  @Log
  public boolean atSetpoint() {
    return err() < 0.025;
  }

  @Log
  public double err() {
    return Math.abs(pos() - pos);
  }

  @Log
  public double pos() {
    return bottomMotor.getSelectedSensorPosition() * TICKS_TO_RAD;
  }

  @Override
  public void periodic() {
    loop.setNextR(pos, 0);
    loop.correct(VecBuilder.fill(bottomMotor.getSelectedSensorPosition() * TICKS_TO_RAD));
    loop.predict(0.02);
    bottomMotor.setVoltage(
        loop.getU(0)
            + Constants.Arm.kS * Math.signum(loop.getNextR(1))
            + Constants.Arm.kG * Math.cos(loop.getNextR(0)));
  }
}
