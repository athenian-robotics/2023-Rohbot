package com.arc852;

import static com.lib.controllers.FightStick.Button.*;
import static com.lib.controllers.Thrustmaster.Button.*;

import com.arc852.autos.PPSwerveCommand;
import com.arc852.subsystems.Arm;
import com.arc852.subsystems.Elevator;
import com.arc852.subsystems.Grabber;
import com.arc852.subsystems.Swerve;
import com.lib.controllers.FightStick;
import com.lib.controllers.Thrustmaster;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements Loggable {
  /* Controllers */

  private static final Thrustmaster stick = new Thrustmaster(0);

  /* Drive Controls */
  private final int leftY = XboxController.Axis.kLeftY.value;
  private final int leftX = XboxController.Axis.kLeftX.value;
  private final int rightX = XboxController.Axis.kRightX.value;

  private final DoubleSubscriber dial =
      NetworkTableInstance.getDefault().getDoubleTopic("phidget/dial").subscribe(0.0);
  private final DoubleSubscriber slider =
      NetworkTableInstance.getDefault().getDoubleTopic("phidget/slider1").subscribe(0.0);
  private final DoubleSubscriber slider2 =
      NetworkTableInstance.getDefault().getDoubleTopic("phidget/slider2").subscribe(0.0);
  private static final FightStick fight = new FightStick(1);
  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  /* Driver Buttons */
  private static final JoystickButton a = new JoystickButton(fight, A.value);
  private static final JoystickButton b = new JoystickButton(fight, B.value);
  private static final JoystickButton x = new JoystickButton(fight, X.value);
  private static final JoystickButton y = new JoystickButton(fight, Y.value);
  private static final JoystickButton lb = new JoystickButton(fight, FightStick.Button.LB.value);
  private static final JoystickButton rb = new JoystickButton(fight, FightStick.Button.RB.value);
  private static final JoystickButton lt = new JoystickButton(fight, FightStick.Button.LT.value);
  private static final JoystickButton rt = new JoystickButton(fight, FightStick.Button.RT.value);
  private static final JoystickButton trigger = new JoystickButton(stick, TRIGGER.val);
  private static final JoystickButton bottom = new JoystickButton(stick, BOTTOM.val);
  private static final JoystickButton leftOutsideTop =
      new JoystickButton(stick, LEFT_OUTSIDE_TOP.val);
  private static final JoystickButton leftOutsideBottom =
      new JoystickButton(stick, LEFT_OUTSIDE_BOTTOM.val);
  private static final JoystickButton leftInsideBottom =
      new JoystickButton(stick, LEFT_INSIDE_BOTTOM.val);

  // just in case flight stick controls

  private static final JoystickButton rightOutsideTop =
      new JoystickButton(stick, RIGHT_OUTSIDE_TOP.val);
  private static final JoystickButton rightOutsideBottom =
      new JoystickButton(stick, RIGHT_OUTSIDE_BOTTOM.val);
  private static final JoystickButton rightInsideBottom =
      new JoystickButton(stick, RIGHT_INSIDE_BOTTOM.val);
  private static final JoystickButton rightInsideTop =
      new JoystickButton(stick, RIGHT_INSIDE_TOP.val);
  private static final JoystickButton rightMiddleBottom =
      new JoystickButton(stick, RIGHT_MIDDLE_BOTTOM.val);
  private static final JoystickButton rightMiddleTop =
      new JoystickButton(stick, RIGHT_MIDDLE_TOP.val);

  /* Subsystems */
  private final Swerve swerve = new Swerve();
  private final Elevator elevator = new Elevator();
  private final Arm arm = new Arm();
  private final Grabber grabber = new Grabber();
  private final Superstructure superstructure = new Superstructure(arm, elevator);
  private final EventLoop loop;

  /* The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(EventLoop loop) {
    this.loop = loop;
    autoChooser.addOption("dump and balance", "dump and balance");
    autoChooser.addOption("dump and leave community", "dump and leave community");
    autoChooser.addOption("mobility dump and balance", "mobility dump and balance");
    autoChooser.addOption("left", "left corner");
    autoChooser.addOption("mid", "middle");
    autoChooser.addOption("right", "right corner");
    autoChooser.addOption("left no balance", "left corner no balance");
    autoChooser.addOption("right no balance", "right corner no balance");
    autoChooser.setDefaultOption("mid", "middle");
    autoChooser.addOption("do nothing", "nothing");
    autoChooser.addOption("back", "drop game piece");
    autoChooser.addOption("place high cone and balance", "place high cone and balance");
    autoChooser.addOption("place high cube", "place high cube");
    arm.set(0);
    SmartDashboard.putData(autoChooser);
    swerve.setDefaultCommand(
        swerve.drive(() -> -stick.getY(), () -> -stick.getX(), () -> -stick.getZ(), () -> false));

    //    elevator.setDefaultCommand(
    //        new InstantCommand(() -> elevator.set(slider.get()), elevator).repeatedly());
    //    arm.setDefaultCommand(new InstantCommand(() -> arm.set(slider2.get()), arm).repeatedly());

    //    swerve.setDefaultCommand(
    //        swerve.drive(
    //            () -> -driver.getRawAxis(translationAxis),
    //            () -> -driver.getRawAxis(strafeAxis),
    //            () -> {
    //              System.out.println(
    //                  Math.atan2(
    //                      -driver.getRawAxis(XboxController.Axis.kRightY.value),
    //                      -driver.getRawAxis(XboxController.Axis.kRightX.value)));
    //              return new Rotation2d(
    //                  Math.atan2(
    //                      MathUtil.applyDeadband(
    //                          -driver.getRawAxis(XboxController.Axis.kRightY.value),
    //                          Constants.stickDeadband),
    //                      MathUtil.applyDeadband(
    //                          -driver.getRawAxis(XboxController.Axis.kRightX.value),
    //                          Constants.stickDeadband)));
    //            }));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */

    a.onTrue(superstructure.highCube());
    b.onTrue(superstructure.midCube());
    x.onTrue(superstructure.highCone());
    y.onTrue(superstructure.midCone());
    lb.onTrue(superstructure.startingPos());
    rb.onTrue(superstructure.low());
    // lb.onTrue(new InstantCommand(swerve::zeroGyro));
    trigger.onTrue(grabber.close());
    bottom.onTrue(grabber.open());
    leftOutsideTop.onTrue(swerve.autoBalance());
    // leftOutsideBottom.onTrue(
    //    new PPSwerveCommand(swerve, true, "chargestation", new PathConstraints(1.3, 1.5)));
    leftInsideBottom.onTrue(new InstantCommand((swerve::zeroGyro)));

    /*
      // just in case controls on flight stick rightOutsideTop.onTrue(superstructure.highCone());
      rightOutsideBottom.onTrue(superstructure.midCone());
      rightMiddleTop.onTrue(superstructure.doubleSubstation());
      rightMiddleBottom.onTrue(superstructure.low());
      rightInsideTop.onTrue(superstructure.ground());
      rightInsideBottom.onTrue(superstructure.startingPos());
     */
    fight.povLeft(loop).castTo(Trigger::new).onTrue(superstructure.doubleSubstation());
    fight.povRight(loop).castTo(Trigger::new).onTrue(superstructure.ground());
    fight.povUp(loop).castTo(Trigger::new).whileTrue(grabber.spinForward());
    fight.povDown(loop).castTo(Trigger::new).whileTrue(grabber.spinBackward());

    //    lb.onTrue(superstructure.startingPos());
    // arm.setDefaultCommand(arm.set((1 - slider2.get()) * -Math.PI).repeatedly());
    // elevator.setDefaultCommand(elevator.set(slider.get()).repeatedly());

  }

  @Log
  public double gert() {
    return slider2.get();
  }

  /*
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomousz
    if (autoChooser.getSelected().equals("nothing")) return new InstantCommand();

    if (autoChooser.getSelected().equals("drop game piece")) {
      return new PPSwerveCommand(
          swerve, true, autoChooser.getSelected(), new PathConstraints(2.0, 1.5));
    }

    if (autoChooser.getSelected().equals("place high cone and balance")) {
      return new SequentialCommandGroup(
          superstructure.highCone(),
          new PPSwerveCommand(swerve, true, "run into terminal", new PathConstraints(2, 1.5)),
          new WaitCommand(1),
          grabber.open(),
          new WaitCommand(0.75),
          superstructure.startingPos(),
          new WaitCommand(0.75),
          new PPSwerveCommand(
              swerve, true, "chargestation backwards", new PathConstraints(1.3, 1.5)));
    }

    if (autoChooser.getSelected().equals("mobility dump and balance")) {
      return new SequentialCommandGroup(
          new PPSwerveCommand(swerve, true, "drop game piece", new PathConstraints(2, 1.5)),
          new PPSwerveCommand(
              swerve, true, "chargestation forwards", new PathConstraints(1.3, 1.5)),
          new PPSwerveCommand(swerve, true, "mobility forwards", new PathConstraints(1.3, 0.7)),
          new WaitCommand(0.3),
          new PPSwerveCommand(swerve, false, "mobility backwards", new PathConstraints(1.3, 1.5)),
          swerve.autoBalance());
    }

    if (autoChooser.getSelected().equals("dump and balance")) {
      return new SequentialCommandGroup(
          new PPSwerveCommand(swerve, true, "drop game piece", new PathConstraints(2, 1.5)),
          new PPSwerveCommand(
              swerve, true, "chargestation forwards", new PathConstraints(1.3, 1.5)),
          swerve.autoBalance());
    }

    if (autoChooser.getSelected().equals("dump and leave community")) {
      return new SequentialCommandGroup(
          new PPSwerveCommand(swerve, true, "drop game piece", new PathConstraints(2, 1.5)),
          new PPSwerveCommand(swerve, true, "leave community", new PathConstraints(1.5, 1.5)));
    }

    return new PPSwerveCommand(
            swerve, true, autoChooser.getSelected(), new PathConstraints(1.5, 1.0))
        .andThen(swerve.autoBalance());
  }
}
