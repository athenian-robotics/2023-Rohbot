package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;

import java.util.Set;

public class Elevator extends SubsystemBase {
    private final WPI_TalonFX leftElevatorMotor;
    private final WPI_TalonFX rightElevatorMotor;
    private final MotorControllerGroup elevatorMotors;

    public Elevator() {
        leftElevatorMotor = new WPI_TalonFX(Constants.ElevatorConstants.LEFT_MOTOR);
        rightElevatorMotor = new WPI_TalonFX(Constants.ElevatorConstants.RIGHT_MOTOR);
        this.elevatorMotors = new MotorControllerGroup(leftElevatorMotor, rightElevatorMotor);
    }

    private void setMotorSpeed(double speed) {
        elevatorMotors.set(speed);
    }

    public Command moveUp(double speed) {
        return new StartEndCommand(
                () -> {
                    if (elevatorMotors.get() < Constants.ElevatorConstants.MAX_HEIGHT) {
                        setMotorSpeed(speed);
                    } else {
                        setMotorSpeed(0);
                    }
                },
                () -> setMotorSpeed(0)
        );
    }

    public Command moveDown(double speed) {
        return new StartEndCommand(
                () -> {
                    if (leftElevatorMotor.getSelectedSensorPosition() > Constants.ElevatorConstants.MIN_HEIGHT) {
                        setMotorSpeed(-speed);
                    } else {
                        setMotorSpeed(0);
                    }
                },
                () -> setMotorSpeed(0)
        );
    }
}

