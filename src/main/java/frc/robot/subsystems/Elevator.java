package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.GenericEntry;
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
    private final GenericEntry position;
    private final GenericEntry velocity;
    private final LinearSystem<N2, N1, N1> sys;
    private final KalmanFilter<N2, N1, N1> filter;
    private final LinearQuadraticRegulator<N2, N1, N1> controller;
    private final LinearSystemLoop<N2, N1, N1> loop;
    private final double  kS = 0;
    
    public Elevator() {
        leftMotor = new WPI_TalonFX(Constants.ElevatorConstants.LEFT_MOTOR);
        rightMotor = new WPI_TalonFX(Constants.ElevatorConstants.RIGHT_MOTOR);
        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setNeutralMode(NeutralMode.Brake);
        this.elevatorMotors = new MotorControllerGroup(leftMotor, rightMotor);
        ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
        position = tab.add("Elevator Position", leftMotor.getSelectedSensorPosition()).getEntry();
        velocity = tab.add("Elevator Speed", leftMotor.getSelectedSensorVelocity()).getEntry();

        sys = LinearSystemId.createElevatorSystem(DCMotor.getFalcon500(2), 0, 0, 0);
        filter = new KalmanFilter<N2, N1, N1>(Nat.N2(), Nat.N1(), sys,
                VecBuilder.fill(0, 0), VecBuilder.fill(0), 0.02);
        controller = new LinearQuadraticRegulator<N2, N1, N1>(sys,
                VecBuilder.fill(0.1, 0.1), VecBuilder.fill(0.1), 0.02);
        loop = new LinearSystemLoop<N2, N1, N1>(sys, controller, filter, 12, 0.02);
    }

    private void setMotorSpeed(double speed) {
        elevatorMotors.set(speed);
    }

    public Command moveUp() {
        return new StartEndCommand(
                () -> {
                    if (leftMotor.getSelectedSensorPosition() > Constants.ElevatorConstants.MAX_HEIGHT) {
                        setMotorSpeed(-0.1);
                    } else {
                        setMotorSpeed(0);
                        System.out.println("Stopped");
                    }
                },
                () -> setMotorSpeed(0)
        );
    }

    public Command moveDown() {
        return new StartEndCommand(
                () -> {
                    if (leftMotor.getSelectedSensorPosition() < Constants.ElevatorConstants.MIN_HEIGHT) {
                        setMotorSpeed(0.1);
                    } else {
                        setMotorSpeed(0);
                        System.out.println("Stopped");
                    }
                },
                () -> setMotorSpeed(0)
        );
    }

    public Command set(double m) {
        
    }
    @Override
    public void periodic(){
        loop.correct(VecBuilder.fill(leftMotor.getSelectedSensorPosition()));
        loop.predict(0.02);
        double volt = loop.getU(0) + kS;

        position.setDouble(leftMotor.getSelectedSensorPosition());
        velocity.setDouble(leftMotor.getSelectedSensorVelocity());
    }
}

