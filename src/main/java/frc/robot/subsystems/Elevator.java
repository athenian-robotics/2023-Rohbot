package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;

import java.util.Set;

public class Elevator extends SubsystemBase {
    private final Encoder elevatorEncoder;
    private final MotorControllerGroup elevatorMotors;

    public Elevator(WPI_TalonFX leftElevatorMotor, WPI_TalonFX rightElevatorMotor, Encoder elevatorEncoder) {
        this.elevatorMotors = new MotorControllerGroup(leftElevatorMotor, rightElevatorMotor);
        this.elevatorEncoder = elevatorEncoder;
    }

    private void setMotorSpeed(double speed) {
        elevatorMotors.set(speed);
    }

    public Command moveUp(double speed) {
        return new StartEndCommand(
                () -> {
                    if (elevatorEncoder.get() < Constants.ElevatorConstants.MAX_HEIGHT) {
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
                    if (elevatorEncoder.get() > Constants.ElevatorConstants.MIN_HEIGHT) {
                        setMotorSpeed(-speed);
                    } else {
                        setMotorSpeed(0);
                    }
                },
                () -> setMotorSpeed(0)
        );
    }
}

