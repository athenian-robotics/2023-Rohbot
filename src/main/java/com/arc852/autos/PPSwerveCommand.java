package com.arc852.autos;

import static com.arc852.Constants.Swerve.swerveKinematics;

import com.arc852.Constants;
import com.arc852.subsystems.Swerve;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PPSwerveCommand extends SequentialCommandGroup {
  public PPSwerveCommand(
      Swerve drivetrain, boolean isFirstPath, String name, PathConstraints constraints) {
    super(
        new InstantCommand(
            () -> {
              // Reset odometry for the first path you run during auto
              if (isFirstPath) {
                drivetrain.resetOdometry(
                    PathPlanner.loadPath(name, constraints).getInitialHolonomicPose());
              }
            }),
        new PPSwerveControllerCommand(
            PathPlanner.loadPath(name, constraints),
            drivetrain::getPose, // Pose supplier
            swerveKinematics, // SwerveDriveKinematics
            new PIDController(
                Constants.Auto.PX_CONTROLLER,
                0,
                0), // X controller. Tune these values for your robot. Leaving them 0 will only use
            // feedforwards.
            new PIDController(
                Constants.Auto.PY_CONTROLLER,
                0,
                0), // Y controller (usually the same values as X controller)
            new PIDController(
                Constants.Auto.P_THETA_CONTROLLER,
                0,
                0), // Rotation controller. Tune these values for your robot. Leaving them 0 will
            // only use feedforwards.
            drivetrain::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color.
            // Optional, defaults to true
            drivetrain // Requires this drive subsystem
            ));
    addRequirements(drivetrain);
  }
}
