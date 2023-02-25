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

public class Arm extends SubsystemBase {
    private final WPI_TalonFX armRotate;
    private final CANSparkMax grabberRotate;
    private final DoubleSolenoid leftSolenoid;
    private final DoubleSolenoid rightSolenoid;

    public Arm() {
        armRotate = new WPI_TalonFX(Constants.ArmConstants.ARM_ROTATE);
        grabberRotate = new CANSparkMax(Constants.ArmConstants.GRABBER_ROTATE, CANSparkMaxLowLevel.MotorType.kBrushless);
        leftSolenoid = new DoubleSolenoid(Constants.ArmConstants.PNEUMATIC_HUB, PneumaticsModuleType.REVPH,
                Constants.ArmConstants.LEFT_SOLENOID_FORWARD, Constants.ArmConstants.LEFT_SOLENOID_REVERSE);
        rightSolenoid = new DoubleSolenoid(Constants.ArmConstants.PNEUMATIC_HUB, PneumaticsModuleType.REVPH,
                Constants.ArmConstants.RIGHT_SOLENOID_FORWARD, Constants.ArmConstants.RIGHT_SOLENOID_REVERSE);
        leftSolenoid.set(DoubleSolenoid.Value.kForward);
        rightSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public Command toggleGrabber() {
        return new InstantCommand(
                () -> {
                    leftSolenoid.toggle();
                    rightSolenoid.toggle();
                },
                this
        );
    }

    public Command moveUp() {
        return new StartEndCommand(
                () -> {
                    if (armRotate.getSelectedSensorPosition() > Constants.ArmConstants.MAX_HEIGHT) {
                        armRotate.set(-0.1);
                    } else {
                        armRotate.set(0);
                        System.out.println("Arm Stopped");
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
                        System.out.println("Arm Stopped");
                    }
                },
                () -> armRotate.set(0)
        );
    }

    public Command spinGrabber() {
        return new StartEndCommand(
                () -> {
                    grabberRotate.setVoltage(0.1);
                },
                () -> grabberRotate.setVoltage(0)
        );
    }
}
