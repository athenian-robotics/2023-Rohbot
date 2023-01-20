// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.arc852;

import com.arc852.inputs.PoseEstimator;
import com.arc852.lib.AutonSelector;
import com.arc852.lib.controllers.ARCXboxController;
import com.arc852.lib.controllers.FightStick;
import com.arc852.lib.limelight.Limelight;
import com.arc852.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import reactor.core.publisher.Flux;
import reactor.util.function.Tuples;

public class RobotContainer {
  public static final ARCXboxController xboxController =
      new ARCXboxController(Constants.OIConstants.xboxControllerPort);
  public static final FightStick fightStick = new FightStick(Constants.OIConstants.fightStickPort);
  public static final AutonSelector autonSelector = new AutonSelector();
  public static final Limelight limelight = new Limelight();
  public static final PoseEstimator estimator = new PoseEstimator(limelight);
  public static final Swerve swerve = new Swerve(estimator, Flux.just());
  public static DriverStation.Alliance alliance = DriverStation.Alliance.Blue;
  final SendableChooser<SequentialCommandGroup> chooser = new SendableChooser<>();

  // Sets up controllers, configures controllers, and sets the default drive mode (tank or arcade)
  public RobotContainer() {

    portForwardLimelightPorts();
    Shuffleboard.getTab("852 - Dashboard")
        .add("Chooser", chooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser);
    bindButtons();
  }

  private void bindButtons() {
    // bind xbox left stick to drive
    swerve.setDriveInput(
        xboxController
            .streamLeftJoystick()
            .zipWith(
                xboxController.streamRightJoystick(),
                (left, right) -> new double[] {left[0], left[1], right[0]})
            .filter((x) -> Robot.getInstance().isTeleop())
            .map(x -> Tuples.of(new Translation2d(-x[1], x[0]), x[2]))); // weird ordering to correct for xbox controller
  }

  public void setAlliance(DriverStation.Alliance alliance) {
    if (alliance != DriverStation.Alliance.Invalid) RobotContainer.alliance = alliance;
  }

  // Returns the robot's main autonomous command
  public Command getAutonomousCommand() {
    return autonSelector.get();
  }

  public void portForwardLimelightPorts() {
    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5802, "limelight.local", 5802);
    PortForwarder.add(5803, "limelight.local", 5803);
    PortForwarder.add(5804, "limelight.local", 5804);
    PortForwarder.add(5805, "limelight.local", 5805);
    PortForwarder.add(5800, "10.8.52.76", 5800);
    PortForwarder.add(5801, "10.8.52.76", 5801);
    PortForwarder.add(5802, "10.8.52.76", 5802);
    PortForwarder.add(5803, "10.8.52.76", 5803);
    PortForwarder.add(5804, "10.8.52.76", 5804);
    PortForwarder.add(5805, "10.8.52.76", 5805);
  }
}
