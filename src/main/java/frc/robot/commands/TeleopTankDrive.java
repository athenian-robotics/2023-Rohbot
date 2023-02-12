package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopTankDrive extends CommandBase {
  private Swerve s_Swerve;
  private XboxController controller;

  public TeleopTankDrive(Swerve s_Swerve, XboxController controller) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.controller = controller;
  }

  @Override
  public void execute() {
    s_Swerve.tankDrive(controller.getLeftY(), controller.getRightY());
  }
}