package com.arc852.lib;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonSelector {

  private final SendableChooser<SequentialCommandGroup> autonomusSelector = new SendableChooser<>();

  public AutonSelector() {
    autonomusSelector.setDefaultOption("DO_NOTHING", new SequentialCommandGroup());

    SmartDashboard.putData("Autonomus Selector", autonomusSelector);
  }

  public SequentialCommandGroup get() {
    return autonomusSelector.getSelected();
  }
}
