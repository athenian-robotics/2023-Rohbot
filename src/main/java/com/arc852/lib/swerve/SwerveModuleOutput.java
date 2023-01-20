package com.arc852.lib.swerve;

import com.arc852.Constants;
import com.arc852.lib.motors.ARCTalonMotor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import reactor.core.publisher.Mono;

public class SwerveModuleOutput {
  // not this one above
  public int moduleNumber;
  private final ARCTalonMotor angleMotor;
  private final ARCTalonMotor driveMotor;

  SimpleMotorFeedforward veloFeedforward =
      new SimpleMotorFeedforward(
          Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  PIDController veloPID = new PIDController(0, 0, 0);
  PIDController anglePID = new PIDController(0, 0, 0);

  public SwerveModuleOutput(SwerveConstants constants) {
    this.moduleNumber = constants.getModuleNumber();
    this.angleMotor = SwerveMotorConfig.createAngleMotor(constants.getAngleMotorID());
    this.driveMotor = SwerveMotorConfig.createDriveMotor(constants.getDriveMotorID());
  }

  public void setDesiredState(
      Mono<SwerveModuleState> desiredState, Mono<SwerveModuleState> currentState) {
    Mono<Double> position;
    Mono<Double> velocity;

    var blocked = currentState.block();
    var curr = blocked == null ? new SwerveModuleState() : blocked;

    // split desired state into position and velocity
    var optimizedDesiredState =
        desiredState.map(state -> SwerveModuleState.optimize(state, curr.angle));

    position = optimizedDesiredState.map(state -> state.angle).map(Rotation2d::getDegrees);
    velocity = optimizedDesiredState.map(state -> state.speedMetersPerSecond);

    position
        .filter(angle -> Math.abs(angle - curr.angle.getDegrees()) >= 0.5)
        .map(angle -> anglePID.calculate(curr.angle.getDegrees(), angle))
        .subscribe(angleMotor::set);

    velocity
        .filter(velo -> Math.abs(velo - curr.speedMetersPerSecond) >= 0.5)
        .map(
            velo ->
                veloFeedforward.calculate(velo, curr.speedMetersPerSecond)
                    + veloPID.calculate(velo, curr.speedMetersPerSecond))
        .subscribe(driveMotor::set);
  }
}
