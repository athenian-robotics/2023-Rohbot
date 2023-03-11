package com.arc852;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.arc852.autos.*;
import com.arc852.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  public static XboxController driver = new XboxController(0);
  private static Joystick leftJoystick = new Joystick(1);
  private static Joystick rightJoystick = new Joystick(2);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton resetGyro = new JoystickButton(driver, XboxController.Button.kX.value); //X button
  private final JoystickButton pathPlanner =
      new JoystickButton(driver, XboxController.Button.kRightBumper.value); //RB button
  private final JoystickButton toggleGrabber =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value); //LB button
  private final JoystickButton spinGrabber =
      new JoystickButton(driver, XboxController.Button.kBack.value); //Back button
  private final JoystickButton startAutoBalancing = new JoystickButton(driver, XboxController.Button.kB.value); //B button
  private final JoystickButton moveElevatorUp = new JoystickButton(driver, XboxController.Button.kY.value); //Y button
  private final JoystickButton moveElevatorDown = new JoystickButton(driver, XboxController.Button.kA.value); //A button
  private final JoystickButton moveArmUp = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value); //LT button
  private final JoystickButton moveArmDown = new JoystickButton(driver, XboxController.Axis.kRightTrigger.value); //RT button

  /* Subsystems */
  private final Swerve swerve = new Swerve();
  private final Elevator elevator = new Elevator();
  private final Arm arm = new Arm();
  private final Grabber grabber = new Grabber();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
            swerve.setDefaultCommand(
                swerve.drive(
                    () -> -driver.getRawAxis(translationAxis),
                    () -> -driver.getRawAxis(strafeAxis),
                    () -> -driver.getRawAxis(rotationAxis),
                        () -> false
                )
            );

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
    resetGyro.onTrue(new InstantCommand(swerve::zeroGyro));
    pathPlanner.onTrue(
        new PPSwerveCommand(
            swerve, true, PathPlanner.loadPath("Move Left", new PathConstraints(1, 1))));
    startAutoBalancing.onTrue(swerve.autoBalance());
    moveElevatorUp.whileTrue(elevator.moveUp());
    moveElevatorDown.whileTrue(elevator.moveDown());
    moveArmUp.whileTrue(arm.moveUp());
    moveArmDown.whileTrue(arm.moveDown());
    toggleGrabber.onTrue(grabber.toggleGrabber());
    spinGrabber.whileTrue(grabber.spinGrabber());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new exampleAuto(swerve);
  }
}
