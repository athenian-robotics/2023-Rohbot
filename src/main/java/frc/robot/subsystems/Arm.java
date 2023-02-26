package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final WPI_TalonFX armRotate;

    public Arm() {
        armRotate = new WPI_TalonFX(Constants.ArmConstants.armMotorID);
    }

    public Command moveUp() {
        return new StartEndCommand(
                () -> {
                    if (armRotate.getSelectedSensorPosition() > Constants.ArmConstants.MAX_HEIGHT) {
                        armRotate.set(-Constants.ArmConstants.ARM_MOVE_SPEED);
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
                        armRotate.set(Constants.ArmConstants.ARM_MOVE_SPEED);
                    } else {
                        armRotate.set(0);
                        System.out.println("Arm Stopped");
                    }
                },
                () -> armRotate.set(0)
        );
    }
}
