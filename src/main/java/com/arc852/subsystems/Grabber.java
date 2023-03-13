package com.arc852.subsystems;

import com.arc852.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {
  private final CANSparkMax motor;
  private final DoubleSolenoid leftSolenoid;
  private final DoubleSolenoid rightSolenoid;
  private final PIDController controller = new PIDController(0.1, 0, 0);
  private double set = 0;

  public Grabber() {
    controller.enableContinuousInput(-Math.PI, Math.PI);
    motor =
        new CANSparkMax(
            Constants.GrabberConstants.grabberSpinMotorID,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    leftSolenoid =
        new DoubleSolenoid(
            Constants.GrabberConstants.PNEUMATIC_HUB,
            PneumaticsModuleType.REVPH,
            Constants.GrabberConstants.LEFT_SOLENOID_FORWARD,
            Constants.GrabberConstants.LEFT_SOLENOID_REVERSE);
    rightSolenoid =
        new DoubleSolenoid(
            Constants.GrabberConstants.PNEUMATIC_HUB,
            PneumaticsModuleType.REVPH,
            Constants.GrabberConstants.RIGHT_SOLENOID_FORWARD,
            Constants.GrabberConstants.RIGHT_SOLENOID_REVERSE);
    leftSolenoid.set(DoubleSolenoid.Value.kForward);
    rightSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public Command openGrabber() {
    return new InstantCommand(
        () -> {
          leftSolenoid.set(DoubleSolenoid.Value.kForward);
          rightSolenoid.set(DoubleSolenoid.Value.kForward);
        });
  }

  public Command closeGrabber() {
    return new InstantCommand(
        () -> {
          leftSolenoid.set(DoubleSolenoid.Value.kReverse);
          rightSolenoid.set(DoubleSolenoid.Value.kReverse);
        });
  }

  /**
   * rotates grabber rot radians
   *
   * @param rot The number of radians to rotate the grabber
   * @return The command to execute.
   */
  public Command spinGrabber(double rot) {
    return new InstantCommand(() -> set = rot, this);
  }

  @Override
  public void periodic() {
    motor.setVoltage(controller.calculate(motor.getEncoder().getPosition(), set));
  }
}
