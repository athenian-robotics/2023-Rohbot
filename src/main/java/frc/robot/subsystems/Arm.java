package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.logging.Level;
import java.util.logging.Logger;

public class Arm extends SubsystemBase {
    private final WPI_TalonFX armRotate;
    private final CANSparkMax grabberRotate;
    private final DoubleSolenoid grabberPistonLeft;
    private final DoubleSolenoid grabberPistonRight;
    private final Logger logger;

    public Arm() {
        armRotate = new WPI_TalonFX(Constants.ArmConstants.ARM_ROTATE);
        grabberRotate = new CANSparkMax(Constants.ArmConstants.GRABBER_ROTATE, CANSparkMaxLowLevel.MotorType.kBrushless);
        grabberPistonLeft = new DoubleSolenoid(Constants.ArmConstants.PNEUMATIC_HUB, PneumaticsModuleType.REVPH,
                Constants.ArmConstants.LEFT_SOLENOID_FORWARD, Constants.ArmConstants.LEFT_SOLENOID_REVERSE);
        grabberPistonRight = new DoubleSolenoid(Constants.ArmConstants.PNEUMATIC_HUB, PneumaticsModuleType.REVPH,
                Constants.ArmConstants.RIGHT_SOLENOID_FORWARD, Constants.ArmConstants.RIGHT_SOLENOID_REVERSE);
        grabberPistonLeft.set(DoubleSolenoid.Value.kForward);
        grabberPistonRight.set(DoubleSolenoid.Value.kForward);
        logger = Logger.getLogger(this.getClass().getName());
    }

    /**
     * Toggles the grabber pistons.
     *
     * @return a command that toggles the grabber pistons
     */
    public Command toggleGrabber() {
        return new InstantCommand(
                () -> {
                    grabberPistonLeft.toggle();
                    grabberPistonRight.toggle();
                },
                this
        );
    }
    /**
     * Moves the arm up until it reaches the maximum height.
     *
     * @return a command that moves the arm up
     */
    public Command moveUp() {
        return new StartEndCommand(
                () -> {
                    if (armRotate.getSelectedSensorPosition() > Constants.ArmConstants.MAX_HEIGHT) {
                        armRotate.set(-0.1);
                    } else {
                        armRotate.set(0);
                        logger.log(Level.INFO, "Arm Stopped");
                    }
                },
                () -> armRotate.set(0)
        );
    }

    public Command moveDown() {
        return new StartEndCommand(
                () -> {
                    if (armRotate.getSelectedSensorPosition() < Constants.ArmConstants.MIN_HEIGHT) {
                        armRotate.set(0.1);
                    } else {
                        armRotate.set(0);
                        logger.log(Level.INFO, "Arm Stopped");
                    }
                },
                () -> armRotate.set(0)
        );
    }
    /**
     * Spins the grabber motor.
     *
     * @return a command that spins the grabber motor
     */
    public Command spinGrabber() {
        return new StartEndCommand(
                () -> {
                    grabberRotate.setVoltage(0.1);
                },
                () -> grabberRotate.setVoltage(0)
        );
    }
}
