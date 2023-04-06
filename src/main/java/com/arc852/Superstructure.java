package com.arc852;

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

  private record Position(double arm, double elevator) {}

  private Position starting = new Position(0,0);

    private Position highCube = new Position(-1.823377330, 0.9568971962588786);
    private Position highCone = new Position(-1.459210181, 0.9218878504588784);
    private Position midCube = new Position(-0.12272477, 0.36684112149532616);
    private Position midCone = new Position(-0.807741689, 0.4943177570093448);
    private Position low = new Position(-0.765973583820, 0);

    private Position state = starting;


//  private static final class Positions {
//    private static final class Starting extends Position {
//      public static final double ARM = 0;
//      public static final double ELEVATOR = 0;
//
//      @Override
//      public double height() {
//        return ELEVATOR;
//      }
//    }
//
//    private static final class HighCube extends Position {
//      public static final double ARM = -1.823377330;
//      public static final double ELEVATOR = 0.9568971962588786;
//
//      @Override
//      public double height() {
//        return ELEVATOR;
//      }
//    }
//
//    private static final class HighCone extends Position {
//      public static final double ARM = -1.459210181;
//      public static final double ELEVATOR = 0.9218878504588784;
//
//      @Override
//      public double height() {
//        return ELEVATOR;
//      }
//    }
//
//    private static final class MidCone extends Position {
//      public static final double ARM = -0.807741689;
//      public static final double ELEVATOR = 0.4943177570093448;
//
//      @Override
//      public double height() {
//        return ELEVATOR;
//      }
//    }
//
//    private static final class Low extends Position {
//      public static final double ARM = -0.765973583820;
//      public static final double ELEVATOR = 0;
//
//      @Override
//      public double height() {
//        return ELEVATOR;
//      }
//    }
//
//    private static final class MidCube extends Position {
//      public static final double ARM = -0.012272477;
//      public static final double ELEVATOR = 0.36684112149532616;
//
//      @Override
//      public double height() {
//        return ELEVATOR;
//      }
//    }
//  }

  /**
   * set the whole manipulator to a position
   *
   * <p>arm is rad elev is percent
   *
   * @return command
   */
  private Command setPos(double arm, double elevator) {
    boolean armFirst = state.elevator() > elevator;

    return armFirst
        ? new ParallelCommandGroup(
                this.arm.set(arm),
                new WaitUntilCommand(this.arm::atSetpoint).andThen(new WaitCommand(0.75)))
            .andThen(this.elevator.set(elevator).until(this.elevator::atSetpoint))
        : new ParallelCommandGroup(
                this.elevator.set(elevator),
                new WaitUntilCommand(this.elevator::atSetpoint).andThen(new WaitCommand(.75)))
            .andThen(this.arm.set(arm))
            .until(this.arm::atSetpoint);
  }

  public Command startingPos() {
    state = starting;
    return setPos(starting.arm(), starting.elevator());
  }

  public Command highCube() {
    state = highCube;
    return setPos(highCube.arm(), highCube.elevator());
  }

  public Command highCone() {
    state = highCone;
    return setPos(highCone.arm(), highCone.elevator());
  }

  public Command midCone() {
    state = midCone;
    return setPos(midCone.arm(), midCone.elevator());
  }

  public Command low() {
    state = low;
    return setPos(low.arm(), low.elevator());
  }

  public Command midCube() {
    state = midCube;
    return setPos(midCube.arm(), midCube.elevator());
  }
}
