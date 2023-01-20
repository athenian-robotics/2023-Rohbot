package com.arc852.lib;

import edu.wpi.first.math.geometry.Pose2d;
import lombok.Getter;

public class TimestampedPose2d {
  @Getter private final double timestamp;
  @Getter private final Pose2d pose;

  public TimestampedPose2d(double timestamp, Pose2d pose) {
    this.timestamp = timestamp;
    this.pose = pose;
  }
}
