package com.arc852.subsystems;

import static com.ctre.phoenix.motorcontrol.NeutralMode.Brake;

import com.arc852.Constants;
import com.arc852.SwerveModule;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Swerve extends SubsystemBase implements Loggable {
  public final SwerveDriveOdometry swerveOdometry;
  public final SwerveModule[] swerveModules;
  public final Pigeon2 gyro;
  private final PIDController pid;
  private final double kP = 0.018;
  private final double kD = 0.001;
  private final GenericEntry pitchEntry;
  private final GenericEntry pEffect;
  private final GenericEntry dEffect;

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          Constants.Auto.P_THETA_CONTROLLER, 0, 0, Constants.Auto.THETA_CONTROLLER_CONSTRAINTS);

  public Swerve() {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    gyro = new Pigeon2(Constants.Swerve.pigeonID, "can");
    gyro.configFactoryDefault();
    zeroGyro();

    swerveModules =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants, "can"),
          new SwerveModule(1, Constants.Swerve.Mod1.constants, "can"),
          new SwerveModule(2, Constants.Swerve.Mod2.constants, "can"),
          new SwerveModule(3, Constants.Swerve.Mod3.constants, "can")
        };

    MotorControllerGroup left =
        new MotorControllerGroup(
            swerveModules[0].getDriveMotor(), swerveModules[2].getDriveMotor());
    MotorControllerGroup right =
        new MotorControllerGroup(
            swerveModules[1].getDriveMotor(), swerveModules[3].getDriveMotor());

    swerveModules[0].getAngleMotor().setNeutralMode(Brake);
    swerveModules[1].getAngleMotor().setNeutralMode(Brake);
    swerveModules[2].getAngleMotor().setNeutralMode(Brake);
    swerveModules[3].getAngleMotor().setNeutralMode(Brake);

    double kI = 0;
    this.pid = new PIDController(kP, kI, kD);
    ShuffleboardTab tab = Shuffleboard.getTab("Swerve");
    tab.add("PID", pid);

    pitchEntry =
        tab.add("Pitch", gyro.getPitch())
            .withWidget(BuiltInWidgets.kGraph)
            .withProperties(Map.of("min", -180, "max", 180))
            .getEntry();

    pEffect =
        tab.add("P Effect", 0)
            .withWidget(BuiltInWidgets.kGraph)
            .withProperties(Map.of("min", -180, "max", 180))
            .getEntry();

    dEffect =
        tab.add("D Effect", 0)
            .withWidget(BuiltInWidgets.kGraph)
            .withProperties(Map.of("min", -180, "max", 180))
            .getEntry();

    /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
     * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
     */
    Timer.delay(1.0);
    resetModulesToAbsolute();

    swerveOdometry =
        new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : swerveModules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : swerveModules) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : swerveModules) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  @Log
  public double getYawDegrees() {
    return getYaw().getDegrees();
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : swerveModules) {
      mod.resetToAbsolute();
    }
  }

  public Command autoBalance() {
    pid.setSetpoint(0);
    pid.setTolerance(3);

    return new RunCommand(
        () -> {
          double power = pid.calculate(gyro.getPitch());
          double translationVal = MathUtil.applyDeadband(-power, Constants.stickDeadband);
          drive(
              new Translation2d(translationVal, 0).times(Constants.Swerve.maxSpeed),
              0,
              false,
              true);
        });
  }

  /*
     Untested

     public Command getAutoBalancing() {
         return new RunCommand(
                 () -> {
                     var v = controller.calculate(
                             getPose(),
                             (mSwerveMods[0].getState().speedMetersPerSecond + mSwerveMods[2].getState().speedMetersPerSecond)/2,
                             (mSwerveMods[1].getState().speedMetersPerSecond + mSwerveMods[3].getState().speedMetersPerSecond)/2,
                             new Pose2d(),
                             0,
                             0
                     );
                     left.setVoltage(v.left);
                     right.setVoltage(v.right);

                     for (SwerveModule mod : mSwerveMods) {
                         mod.setAngle(new SwerveModuleState(42069, Rotation2d.fromDegrees(0))); // dummy value to bypass jitter checks in setangle
                     }

                     drive.feed();
                 },
                 this
         ).until(() -> -Math.abs(gyro.getPitch())<=0.5);
     }
  */

  public Command drive(
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup) {
    SlewRateLimiter translationLimiter = new SlewRateLimiter(2);
    SlewRateLimiter strafeLimiter = new SlewRateLimiter(2);

    return new RunCommand(
        () -> {
          double translationVal =
              translationLimiter.calculate(
                  MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband));
          double strafeVal =
              strafeLimiter.calculate(
                  MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband));
          double rotationVal =
              MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

          /* Drive */
          drive(
              new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
              rotationVal * Constants.Swerve.maxAngularVelocity,
              !robotCentricSup.getAsBoolean(),
              true);
        },
        this);
  }

  public Command drive(
      DoubleSupplier translationSup, DoubleSupplier strafeSup, Supplier<Rotation2d> rohtation) {
    SlewRateLimiter translationLimiter = new SlewRateLimiter(2);
    SlewRateLimiter strafeLimiter = new SlewRateLimiter(2);

    return new InstantCommand(
            () -> {
              thetaController.reset(getPose().getRotation().getRadians());
              thetaController.setGoal(rohtation.get().getRadians());
              thetaController.setTolerance(0.05);
            })
        .andThen(
            new RunCommand(
                () -> {
                  double translationVal =
                      translationLimiter.calculate(
                          MathUtil.applyDeadband(
                              translationSup.getAsDouble(), Constants.stickDeadband));
                  double strafeVal =
                      strafeLimiter.calculate(
                          MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband));
                  SmartDashboard.putNumber("rot", getYaw().getDegrees());
                  SmartDashboard.putNumber("set", Math.toDegrees(pid.getSetpoint()));
                  double thetadot = thetaController.calculate(getYaw().getRadians());
                  drive(
                      new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                      thetadot,
                      true,
                      true);
                }));
  }

  public Command lockWheels() {
    return new InstantCommand(
            () -> {
              for (SwerveModule mods : swerveModules) {
                mods.setDesiredState(new SwerveModuleState(0, new Rotation2d(Math.PI / 4)), true);
              }
            })
        .alongWith(new WaitCommand(3.5));
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getModulePositions());

    SmartDashboard.putNumber("Pitch", gyro.getPitch());
    SmartDashboard.putNumber("rot", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("setpoint", Math.toDegrees(pid.getSetpoint()));
    pitchEntry.setDouble(gyro.getPitch());
    pEffect.setDouble(kP * pid.getPositionError());
    dEffect.setDouble(kD * pid.getVelocityError());

    for (SwerveModule mod : swerveModules) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
}
