package com.arc852;

import static com.lib.controllers.FightStick.Button.*;

import com.arc852.autos.*;
import com.arc852.subsystems.*;
import com.lib.controllers.FightStick;
import com.lib.controllers.Thrustmaster;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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

  private static Thrustmaster stick = new Thrustmaster(0);

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
  private static final JoystickButton bot =
      new JoystickButton(stick, Joystick.ButtonType.kTop.value);
  /* Subsystems */
  private final Swerve swerve = new Swerve();
  private final Elevator elevator = new Elevator();
  private final Arm arm = new Arm();
  private final Grabber grabber = new Grabber();

  /* The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(EventLoop loop) {
    autoChooser.addOption("left", "left corner");
    autoChooser.addOption("mid", "middle");
    autoChooser.addOption("right", "right corner");
    autoChooser.setDefaultOption("mid", "middle");

    swerve.setDefaultCommand(
        swerve.drive(() -> -stick.getX(), () -> -stick.getY(), () -> -stick.getZ(), () -> false));

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
    //    x.onTrue(new InstantCommand(swerve::zeroGyro));
    //    RB.onTrue(
    //        new PPSwerveCommand(
    //            swerve, true, PathPlanner.loadPath("Move Left", new PathConstraints(1, 1))));
    //    b.onTrue(swerve.autoBalance());
    //    a.onTrue(grabber.open());
    //    y.onTrue(grabber.close());
    //    y.whileTrue(elevator.moveUp());
    //    a.whileTrue(elevator.moveDown());
    //    leftTrigger.whileTrue(arm.moveUp());
    //    rightTrigger.whileTrue(arm.moveDown());
    //    LB.onTrue(grabber.toggleGrabber());
    //    back.whileTrue(grabber.spinGrabber());
    //    new BooleanEvent(loop, fight::getAButton).ifHigh(grabber::open);
    //    //    new BooleanEvent(loop, fight::getBButton).ifHigh(grabber::close);

    a.onTrue(grabber.open());
    b.onTrue(grabber.close());
    x.whileTrue(grabber.spinForward());
    y.whileTrue(grabber.spinBackward());
    bot.onTrue(new InstantCommand(swerve::zeroGyro));
    //    a.onTrue(
    //        new InstantCommand(
    //            () -> {
    //              System.out.println("a");
    //            },
    //            grabber));
    //    b.onTrue(new InstantCommand(() -> System.out.println("b"), grabber));

    arm.setDefaultCommand(arm.set(() -> (1 - slider2.get()) * -Math.PI));
    elevator.setDefaultCommand(elevator.set(slider));
  }

  @Log
  public double gert() {
    return slider2.get();
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomousz
    return new PPSwerveCommand(
            swerve, true, autoChooser.getSelected(), new PathConstraints(1.5, 1.0))
        .andThen(swerve.autoBalance());
  }
}
