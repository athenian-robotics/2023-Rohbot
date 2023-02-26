package frc.robot.autos;

import static frc.robot.Constants.Swerve.swerveKinematics;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class PPSwerveCommand extends SequentialCommandGroup {
  public PPSwerveCommand(Swerve drivetrain, boolean isFirstPath, PathPlannerTrajectory traj) {
    super(
        new InstantCommand(
            () -> {
              // Reset odometry for the first path you run during auto
              if (isFirstPath) {
                drivetrain.resetOdometry(traj.getInitialHolonomicPose());
              }
            }),
        new PPSwerveControllerCommand(
            traj,
            drivetrain::getPose, // Pose supplier
            swerveKinematics, // SwerveDriveKinematics
            new PIDController(
                0, 0,
                0), // X controller. Tune these values for your robot. Leaving them 0 will only use
            // feedforwards.
            new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(
                0, 0,
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
