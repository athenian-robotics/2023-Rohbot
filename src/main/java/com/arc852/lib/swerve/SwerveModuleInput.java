package com.arc852.lib.swerve;

import com.arc852.lib.encoders.ARCCANCoder;
import com.arc852.lib.motors.ARCTalonEncoder;
import com.arc852.lib.motors.ARCTalonManager;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import lombok.Getter;
import reactor.core.publisher.Flux;

public class SwerveModuleInput {
  @Getter private final Flux<SwerveModuleState> state;
  private final ARCTalonEncoder driveEncoder;

  // TODO: make cancoder reactive
  private final ARCCANCoder angleEncoder;

  // TODO: move to constants
  private static final double DRIVE_GEARING = 0;
  private static final double WHEEL_RADIUS = 0;
  private static final double DRIVE_ENCODER_TO_METERS =
      DRIVE_GEARING * WHEEL_RADIUS * 2 * Math.PI / 2048;

  private final double angleOffset;

  public SwerveModuleInput(SwerveConstants constants) {
    this.driveEncoder = ARCTalonManager.createTalonEncoder(constants.getDriveMotorID());
    this.angleEncoder = new ARCCANCoder(constants.getCanCoderID());
    angleEncoder.configFactoryDefault();
    angleEncoder.configAllSettings(SwerveMotorConfig.swerveCanCoderConfig);
    angleOffset = constants.getAngleOffset();

    // this is kind of a hack. it takes advantage of the fact that the angle encoder isnt reactive
    // HINT: use .zipWith() to combine the two
    state =
        driveEncoder
            .getSelectedSensorVelocityFlux()
            .zipWith(
                angleEncoder.getPosition(),
                (speed, angle) ->
                    new SwerveModuleState(
                        speed, new Rotation2d(Math.toRadians(angle / 4096 * 360 + angleOffset))));
  }

  public Flux<SwerveModulePosition> getSwerveModulePosition() {
    return driveEncoder
        .getSelectedSensorPositionFlux()
        .zipWith(
            angleEncoder.getPosition(),
            (pos, ang) ->
                new SwerveModulePosition(
                    pos * DRIVE_ENCODER_TO_METERS,
                    new Rotation2d(Math.toRadians(ang / 4096 * 360 + angleOffset))));
  }

  public void resetToAbsoslute() {
    angleEncoder.setPositionToAbsolute();
  }
}
