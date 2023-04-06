package com.arc852;

import static com.arc852.Superstructure.ARM_OR_ELEVATOR.ARM;

import com.arc852.subsystems.Arm;
import com.arc852.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Superstructure {
  private final Arm arm;
  //  private final Led led;
  private final Elevator elevator;

  public Superstructure(Arm arm, Elevator elevator) {
    this.arm = arm;
    //    this.led = led;
    this.elevator = elevator;
  }

  private static final class Positions {
    private static final class Starting {
      public static final double ARM = 0;
      public static final double ELEVATOR = 0;
    }
  }
  /**
   * set the whole manipulator to a position
   *
   * <p>arm is rad elev is percent
   *
   * @return command
   */
  private Command setPos(
      double arm, double elevator, double timeToStay, ARM_OR_ELEVATOR armOrElevatorFirst) {
    return (armOrElevatorFirst == ARM
            ? new ParallelCommandGroup(
                    this.arm.set(() -> arm),
                    new WaitUntilCommand(this.arm::atSetpoint).andThen(new WaitCommand(0.75)))
                .andThen(this.elevator.set(() -> elevator).until(this.elevator::atSetpoint))
            : new ParallelCommandGroup(
                    this.elevator.set(() -> elevator),
                    new WaitUntilCommand(this.elevator::atSetpoint).andThen(new WaitCommand(.75)))
                .andThen(this.arm.set(() -> arm))
                .until(this.arm::atSetpoint))
        .andThen(new WaitCommand(timeToStay));
  }

  public Command startingPos() {
    return setPos(Positions.Starting.ARM, Positions.Starting.ELEVATOR, 2
            , ARM);
  }

  enum ARM_OR_ELEVATOR {
    ELEVATOR,
    ARM
  }
}
