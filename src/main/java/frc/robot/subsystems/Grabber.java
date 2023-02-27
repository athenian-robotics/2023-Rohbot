package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Grabber extends SubsystemBase {
  private final CANSparkMax grabberRotate;
  private final DoubleSolenoid leftSolenoid;
  private final DoubleSolenoid rightSolenoid;

  public Grabber() {
    grabberRotate =
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

  /**
   * Toggles the grabber solenoid.
   *
   * When the grabber is open (not grabbing), this command closes the grabber.
   * When the grabber is closed (grabbing), this command opens the grabber.
   *
   * @return The command to execute.
   */
  public Command toggleGrabber() {
    return new InstantCommand(
        () -> {
          leftSolenoid.toggle();
          rightSolenoid.toggle();
        },
        this);
  }

  /**
   * This command spins the grabber motor at a constant speed. 
   */
  public Command spinGrabber() {
    return new StartEndCommand(
        () -> grabberRotate.set(Constants.GrabberConstants.SPIN_SPEED), () -> grabberRotate.set(0));
  }
}
