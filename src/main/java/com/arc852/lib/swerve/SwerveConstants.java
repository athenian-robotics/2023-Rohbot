package com.arc852.lib.swerve;

import lombok.Data;

@Data
public class SwerveConstants {
  private final int moduleNumber;
  private final double angleOffset;
  private final int driveMotorID;
  private final int angleMotorID;
  private final int canCoderID;
}
