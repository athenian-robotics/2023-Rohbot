package com.arc852;

import com.arc852.subsystems.Arm;
import com.arc852.subsystems.Elevator;
import com.arc852.subsystems.Grabber;
import com.arc852.subsystems.Led;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class Superstructure {
  private final Arm arm;
  private final Grabber grabber;
  private final Led led;
  private final Elevator elevator;

  public Superstructure(Arm arm, Grabber grabber, Led led, Elevator elevator) {
    this.arm = arm;
    this.grabber = grabber;
    this.led = led;
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
  private Command setPos(double arm, double elevator) {
    return new ParallelCommandGroup(this.arm.set(() -> arm), this.elevator.set(() -> elevator));
  }

  public Command startingPos() {
    return setPos(Positions.Starting.ARM, Positions.Starting.ELEVATOR);
  }
}
