package com.arc852.subsystems;

import com.arc852.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {
  private final CANSparkMax motor;
  private final Solenoid leftSolenoid;
  private final PIDController controller = new PIDController(0.1, 0, 0);
  private final Solenoid rightSolenoid;
  private double set = 0;

  public Grabber() {
    controller.enableContinuousInput(-Math.PI, Math.PI);
    motor =
        new CANSparkMax(
            Constants.Grabber.grabberSpinMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftSolenoid =
        new Solenoid(0, PneumaticsModuleType.CTREPCM, Constants.Grabber.LEFT_SOLENOID_FORWARD);

    rightSolenoid =
        new Solenoid(0, PneumaticsModuleType.CTREPCM, Constants.Grabber.RIGHT_SOLENOID_FORWARD);

    leftSolenoid.set(true);
    rightSolenoid.set(true);
  }

  public Command open() {
    return new InstantCommand(
        () -> {
          System.out.println("a");
          leftSolenoid.set(true);
          rightSolenoid.set(false);
        });
  }

  public Command close() {
    return new InstantCommand(
        () -> {
          System.out.println("b");
          leftSolenoid.set(false);
          rightSolenoid.set(true);
        });
  }

  /**
   * rotates grabber rot radians
   *
   * @param rot The number of radians to rotate the grabber
   * @return The command to execute.
   */
  public Command spin(double rot) {
    return new InstantCommand(() -> set = rot, this);
  }

  @Override
  public void periodic() {
    motor.setVoltage(controller.calculate(motor.getEncoder().getPosition(), set));
  }
}
