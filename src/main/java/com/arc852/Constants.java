package com.arc852;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.lib.util.COTSFalconSwerveConstants;
import com.lib.util.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final double stickDeadband = 0.1;
  public static final double driveGearRatio = (6.75 / 1.0);

  public static double kv = 0.16037;
  public static double ka = 0.20799;
  public static double trackWidthMeters = Units.inchesToMeters(30);

  public static final class Swerve {
    public static final int pigeonID = 13;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants chosenModule =
        COTSFalconSwerveConstants.SDSMK4(Constants.driveGearRatio);

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(23.75);
    public static final double wheelBase = Units.inchesToMeters(23.75);
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /* Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = (0.667 / 12); // TODO: This must be tuned to specific robot
    public static final double driveKV = (2.44 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.5; // TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity = 3; // TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(286.34765);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(168.22265);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 9;
      public static final int angleMotorID = 10;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(92.16406);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 12;
      public static final int canCoderID = 4;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(349.27734);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  public static final class Elevator { // TODO: MAX_HEIGHT must be tuned further
    public static final double MIN_HEIGHT = -6000.0 / 2048 / 6;
    public static final double MAX_HEIGHT = -60000.0 / 2048 / 6;

    public static final int LEFT_MOTOR = 14;

    public static final int RIGHT_MOTOR = 15;

    public static final double kV = 2.3581;
    public static final double kA = 0.072871;
    public static final double kS = 0.41994;
    public static final double kG = .20;

    public static final double TICKS_TO_METERS =
        1.0 / 2048 / 6; // fake unit, around 10 rotatoins or 20k ticks
  }

  public static final class Arm { // TODO: Replace dummy values
    public static final int LEFT_MOTOR = 16;

    public static final double kV = 6.0142;
    public static final double kA = 0.17752;
    public static final double kS = 0.06012;

    public static final double MIN_ANGLE = -Math.PI;
    public static final double MAX_ANGLE = 0;
    public static final double kG = 0.21141;

    public static final double gearRatio = 350;
  }

  public static final class LedConstants {
    public static final int LED_PORT = 0;
  }

  public static final class Grabber {
    public static final int grabberSpinMotorID = 17;
    public static final int SOLENOID_LEFT = 6;
    public static final int SOLENOID_RIGHT = 7;
  }

  public static final
  class Auto { // TODO: The below constants are used in the example auto, and must be tuned
    // to specific robot
    public static final double MAX_SPEED_METERS_PER_SECOND = 3;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

    public static final double PX_CONTROLLER = 0;
    public static final double PY_CONTROLLER = 0;
    public static final double P_THETA_CONTROLLER = 0;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
        new TrapezoidProfile.Constraints(
            MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
  }
}
