package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;

import java.util.Set;

public class Elevator extends SubsystemBase {
    private final WPI_TalonFX leftMotor;
    private final WPI_TalonFX rightMotor;
    private final MotorControllerGroup elevatorMotors;

    public Elevator() {
        leftMotor = new WPI_TalonFX(Constants.ElevatorConstants.LEFT_MOTOR);
        rightMotor = new WPI_TalonFX(Constants.ElevatorConstants.RIGHT_MOTOR);
        this.elevatorMotors = new MotorControllerGroup(leftMotor, rightMotor);
        ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
        tab.add("Elevator Position", leftMotor.getSelectedSensorPosition());
        tab.add("Elevator Speed", leftMotor.getSelectedSensorVelocity());
    }

    private void setMotorSpeed(double speed) {
        elevatorMotors.set(speed);
    }

    public Command moveUp() {
        return new StartEndCommand(
                () -> {
                    if (leftMotor.getSelectedSensorPosition() < Constants.ElevatorConstants.MAX_HEIGHT) {
                        setMotorSpeed(0.1);
                    } else {
                        setMotorSpeed(0);
                    }
                },
                () -> setMotorSpeed(0)
        );
    }

    public Command moveDown() {
        return new StartEndCommand(
                () -> {
                    if (leftMotor.getSelectedSensorPosition() > Constants.ElevatorConstants.MIN_HEIGHT) {
                        setMotorSpeed(-0.1);
                    } else {
                        setMotorSpeed(0);
                    }
                },
                () -> setMotorSpeed(0)
        );
    }
    public void periodic(){

    }
}

