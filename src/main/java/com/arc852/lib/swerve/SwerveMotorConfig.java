package com.arc852.lib.swerve;

import com.arc852.Constants;
import com.arc852.lib.motors.ARCTalonManager;
import com.arc852.lib.motors.ARCTalonMotor;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class SwerveMotorConfig {
  public static TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
  public static TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
  public static CANCoderConfiguration swerveCanCoderConfig = new CANCoderConfiguration();

  public SwerveMotorConfig() {
    /* Swerve Angle Motor Configurations */
    SupplyCurrentLimitConfiguration angleSupplyLimit =
        new SupplyCurrentLimitConfiguration(
            Constants.Swerve.angleEnableCurrentLimit,
            Constants.Swerve.angleContinuousCurrentLimit,
            Constants.Swerve.anglePeakCurrentLimit,
            Constants.Swerve.anglePeakCurrentDuration);

    swerveAngleFXConfig.slot0.kP = Constants.Swerve.angleKP;
    swerveAngleFXConfig.slot0.kI = Constants.Swerve.angleKI;
    swerveAngleFXConfig.slot0.kD = Constants.Swerve.angleKD;
    swerveAngleFXConfig.slot0.kF = Constants.Swerve.angleKF;
    swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
    swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

    /* Swerve Drive Motor Configuration */
    SupplyCurrentLimitConfiguration driveSupplyLimit =
        new SupplyCurrentLimitConfiguration(
            Constants.Swerve.driveEnableCurrentLimit,
            Constants.Swerve.driveContinuousCurrentLimit,
            Constants.Swerve.drivePeakCurrentLimit,
            Constants.Swerve.drivePeakCurrentDuration);

    swerveDriveFXConfig.slot0.kP = Constants.Swerve.driveKP;
    swerveDriveFXConfig.slot0.kI = Constants.Swerve.driveKI;
    swerveDriveFXConfig.slot0.kD = Constants.Swerve.driveKD;
    swerveDriveFXConfig.slot0.kF = Constants.Swerve.driveKF;
    swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
    swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
    swerveDriveFXConfig.openloopRamp = Constants.Swerve.openLoopRamp;
    swerveDriveFXConfig.closedloopRamp = Constants.Swerve.closedLoopRamp;

    /* Swerve CANCoder Configuration */
    swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
    swerveCanCoderConfig.initializationStrategy =
        SensorInitializationStrategy.BootToAbsolutePosition;
    swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
  }

  public static ARCTalonMotor createAngleMotor(int deviceNumber) {
    var angleMotor = ARCTalonManager.createTalonMotor(deviceNumber);
    angleMotor.configAllSettings(SwerveMotorConfig.swerveAngleFXConfig);
    return angleMotor;
  }

  public static ARCTalonMotor createDriveMotor(int deviceNumber) {
    var driveMotor = ARCTalonManager.createTalonMotor(deviceNumber);
    driveMotor.configAllSettings(SwerveMotorConfig.swerveDriveFXConfig);
    return driveMotor;
  }
}
