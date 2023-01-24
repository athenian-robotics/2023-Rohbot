package com.arc852.inputs;

import com.arc852.Constants;
import com.arc852.lib.limelight.Limelight;
import com.arc852.lib.swerve.SwerveModuleInput;
import com.ctre.phoenix.sensors.Pigeon2;
import com.google.common.util.concurrent.ListenableFuture;
import com.google.common.util.concurrent.MoreExecutors;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.Arrays;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import reactor.core.publisher.Flux;
import reactor.core.publisher.Sinks;

public class PoseEstimator {
  private final Pigeon2 gyro = new Pigeon2(Constants.pigeonPort);
  private static final Logger logger = LogManager.getLogger();
  private final SwerveDrivePoseEstimator estimator;

  private final SwerveModuleInput frontLeft;
  private final SwerveModuleInput frontRight;
  private final SwerveModuleInput backLeft;
  private final SwerveModuleInput backRight;
  Sinks.Many<Pose2d> poseSink = Sinks.many().multicast().onBackpressureBuffer();

  private volatile SwerveModulePosition[] states = null;

  public PoseEstimator(Limelight limelight) {
    this.frontLeft = new SwerveModuleInput(Constants.Swerve.mod0);
    this.frontRight = new SwerveModuleInput(Constants.Swerve.mod1);
    this.backLeft = new SwerveModuleInput(Constants.Swerve.mod2);
    this.backRight = new SwerveModuleInput(Constants.Swerve.mod3);

    Flux<SwerveModulePosition[]> positions =
        frontLeft
            .getSwerveModulePosition()
            .zipWith(
                frontRight.getSwerveModulePosition(), (a, b) -> new SwerveModulePosition[] {a, b})
            .zipWith(
                backLeft.getSwerveModulePosition(),
                (a, b) -> new SwerveModulePosition[] {a[0], a[1], b})
            .zipWith(
                backRight.getSwerveModulePosition(),
                (a, b) -> new SwerveModulePosition[] {a[0], a[1], a[2], b});

    var sub = positions.subscribe(x -> states = x);
    Runnable task =
        () -> {
          while (states == null) {
            if (Thread.interrupted()) {
              break;
            }
            Thread.onSpinWait();
          }
        };
    ExecutorService executor = Executors.newSingleThreadExecutor();
    // 500 ms timeout
    ListenableFuture<?> future = MoreExecutors.listeningDecorator(executor).submit(task);
    try {
      future.get(500, TimeUnit.MILLISECONDS);
    } catch (Exception e) {
      logger.error("Failed to get swerve module positions", e);
    }
    executor.shutdown();

    if (states == null) {
      states =
          new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
          };
    }

    sub.dispose();

    estimator =
        new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            new Rotation2d(gyro.getYaw()),
            states,
            new Pose2d() // TODO: set to auto stuff
            );

    // TODO: instead of exception switch to robot oriented drive

    Flux.zip(
            (Object[] array) ->
                Arrays.copyOf(
                    array, array.length, SwerveModulePosition[].class), // stupid api limitation
            frontLeft.getSwerveModulePosition(),
            frontRight.getSwerveModulePosition(),
            backLeft.getSwerveModulePosition(),
            backRight.getSwerveModulePosition())
        .subscribe(
            states -> {
              estimator.update(new Rotation2d(gyro.getYaw()), states);
              poseSink.tryEmitNext(estimator.getEstimatedPosition());
            });

    limelight
        .getVisionPose()
        .subscribe(pose -> estimator.addVisionMeasurement(pose.getPose(), pose.getTimestamp()));
  }

  public Flux<SwerveModuleState> getFrontLeftState() {
    return frontLeft.getState();
  }

  public Flux<SwerveModuleState> getFrontRightState() {
    return frontRight.getState();
  }

  public Flux<SwerveModuleState> getBackLeftState() {
    return backLeft.getState();
  }

  public Flux<SwerveModuleState> getBackRightState() {
    return backRight.getState();
  }

  public Flux<Pose2d> getPose() {
    return poseSink.asFlux().repeat();
  }
}
