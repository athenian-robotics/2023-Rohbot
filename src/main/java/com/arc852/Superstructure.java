package com.arc852;

import com.arc852.subsystems.Arm;
import com.arc852.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Superstructure implements Loggable {
  private final Arm arm;
  //  private final Led led;
  private final Elevator elevator;

  public Superstructure(Arm arm, Elevator elevator) {
    this.arm = arm;
    //    this.led = led;
    this.elevator = elevator;
  }

  private record Position(double arm, double elevator) {

    public boolean equals(Position pos) {
      return this.arm() == pos.arm() && this.elevator() == pos.elevator();
    }
  }

  private static final Position starting = new Position(0, 0);

  private static final Position highCube = new Position(-1.823377330, 0.9568971962588786);
  private static final Position highCone =
      new Position(-1.459210181, 0.9718878504588784); // (-1.459210181, 0.9218878504588784)
  private static final Position midCube = new Position(-0.959211335, 0.36684112149532616);
  private static final Position midCone = new Position(-0.807741689, 0.4943177570093448);
  private static final Position low = new Position(-0.765973583820, 0);
  private static final Position doubleSubstation =
      new Position(-1.510776248, 0.7556915887850468); // (-1.610776248, 0.7156915887850468)
  private static final Position ground =
      new Position(-2.8823904069420, 0.462803738317757); // -3.1423904069420, 0.562803738317757)

  @Log.ToString private Position state = starting;

  /**
   * set the whole manipulator to a position
   *
   * <p>arm is rad elev is percent
   *
   * @return command
   */
  private Command setPos(Position pos) {
    boolean armFirst = state.elevator() > pos.elevator();
    double arm = pos.arm();
    double elevator = pos.elevator();

    if (state.equals(ground)) armFirst = true;
    if (state.equals(starting)) armFirst = false;
    if (state.equals(low)) armFirst = false;
    state = pos;

    //    System.out.println("state: " + state + "\n pos: " + pos + "\n armFirst" + armFirst);
    //    if (state.equals(low)) {
    //      System.out.println("state: " + state + "\n pos: " + pos + "\n armFirst" + armFirst);
    //      return new InstantCommand();
    //    }

    return armFirst
        ? this.arm
            .set(arm)
            .andThen(
                new WaitCommand(0.5),
                this.elevator.set(elevator),
                new WaitUntilCommand(() -> this.arm.atSetpoint() && this.elevator.atSetpoint()))
        : this.elevator
            .set(elevator)
            .andThen(
                new WaitCommand(0.5),
                this.arm.set(arm),
                new WaitUntilCommand(() -> this.arm.atSetpoint() && this.elevator.atSetpoint()));
  }

  public Command startingPos() {
    return setPos(starting);
  }

  public Command highCube() {
    return setPos(highCube);
  }

  public Command highCone() {
    state = highCone;
    return setPos(highCone);
  }

  public Command midCone() {
    return setPos(midCone);
  }

  public Command low() {
    return setPos(low);
  }

  public Command midCube() {
    return setPos(midCube);
  }

  public Command doubleSubstation() {
    return setPos(doubleSubstation);
  }

  public Command ground() {
    return setPos(ground);
  }
}
