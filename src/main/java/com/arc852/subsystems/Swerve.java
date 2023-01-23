package com.arc852.subsystems;

import com.arc852.Constants;
import com.arc852.inputs.PoseEstimator;
import com.arc852.lib.swerve.SwerveModuleOutput;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import reactor.core.Disposable;
import reactor.core.Disposables;
import reactor.core.publisher.Flux;
import reactor.core.publisher.Mono;
import reactor.util.function.Tuple2;

public class Swerve extends SubsystemBase {
  private static final Logger logger = LogManager.getLogger();
  private final SwerveModuleOutput frontLeft = new SwerveModuleOutput(Constants.Swerve.mod0);
  private final SwerveModuleOutput frontRight = new SwerveModuleOutput(Constants.Swerve.mod1);
  private final SwerveModuleOutput backLeft = new SwerveModuleOutput(Constants.Swerve.mod2);
  private final SwerveModuleOutput backRight = new SwerveModuleOutput(Constants.Swerve.mod3);

  private final Disposable.Swap swerveSubscription = Disposables.swap();
  private final PoseEstimator estimator;

  public Swerve(PoseEstimator estimator, Flux<Tuple2<Translation2d, Double>> driveInput) {
    this.estimator = estimator;
    if (!swerveSubscription.update(driveInput.subscribe(this::drive)))
      logger.error("Failed to update init drive " + "input");
  }

  // you might be able to actually use this to map a Flux<Tuple2<Translation2d>>, Double>> to a
  // Flux<Tuple4<Mono<SwerveModuleState>>> and then call .subscribe()
  public void drive(Tuple2<Translation2d, Double> input) {
    var robotPose = estimator.getPose().blockFirst();
    if (robotPose == null) logger.error("robot pose is somehow null, pose estimator is broken");
    var robotRotation = robotPose == null ? new Rotation2d() : robotPose.getRotation();

    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                input.getT1().getX(), input.getT1().getY(), input.getT2(), robotRotation));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    frontLeft.setDesiredState(
        Mono.just(swerveModuleStates[0]), estimator.getFrontLeftState().last());
    frontRight.setDesiredState(
        Mono.just(swerveModuleStates[1]), estimator.getFrontRightState().last());
    backLeft.setDesiredState(Mono.just(swerveModuleStates[2]), estimator.getBackLeftState().last());
    backRight.setDesiredState(
        Mono.just(swerveModuleStates[3]), estimator.getBackRightState().last());
  }

  public void setDriveInput(Flux<Tuple2<Translation2d, Double>> driveInput) {
    if (!swerveSubscription.update(driveInput.subscribe(this::drive)))
      logger.error("failed to update " + "driving subscription");
  }
}
