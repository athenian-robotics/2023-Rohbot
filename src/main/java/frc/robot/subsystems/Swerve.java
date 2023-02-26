package frc.robot.subsystems;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LTVDifferentialDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.SwerveModule;
import frc.robot.Constants;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.autos.PPSwerveCommand;

import java.util.List;
import java.util.Map;

import static com.ctre.phoenix.motorcontrol.NeutralMode.Brake;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    private MotorControllerGroup left;
    private MotorControllerGroup right;
    private final DifferentialDrive drive;
    private final PIDController pid;
    private final double kP = 0.018;
    private final double kI = 0;
    private final double kD = 0.001;
    private final GenericEntry pitchEntry;
    private final GenericEntry pEffect;
    private final GenericEntry dEffect;


    private final LTVDifferentialDriveController controller;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        left = new MotorControllerGroup(mSwerveMods[0].getDriveMotor(), mSwerveMods[2].getDriveMotor());
        right = new MotorControllerGroup(mSwerveMods[1].getDriveMotor(), mSwerveMods[3].getDriveMotor());

        mSwerveMods[0].getAngleMotor().setNeutralMode(Brake);
        mSwerveMods[1].getAngleMotor().setNeutralMode(Brake);
        mSwerveMods[2].getAngleMotor().setNeutralMode(Brake);
        mSwerveMods[3].getAngleMotor().setNeutralMode(Brake);

        drive = new DifferentialDrive(left, right);
        this.pid = new PIDController(kP, kI, kD);
        ShuffleboardTab tab = Shuffleboard.getTab("Swerve");
        tab.add("PID", pid);

        pitchEntry = tab.add("Pitch", gyro.getPitch())
                .withWidget(BuiltInWidgets.kGraph)
                .withProperties(Map.of("min", -180, "max", 180))
                .getEntry();

        pEffect = tab.add("P Effect", 0)
                .withWidget(BuiltInWidgets.kGraph)
                .withProperties(Map.of("min", -180, "max", 180))
                .getEntry();

        dEffect = tab.add("D Effect", 0)
                .withWidget(BuiltInWidgets.kGraph)
                .withProperties(Map.of("min", -180, "max", 180))
                .getEntry();

        controller = new LTVDifferentialDriveController(
                LinearSystemId.identifyDrivetrainSystem(
                        Constants.kv,
                        Constants.ka,
                        Constants.kvRot,
                        Constants.kaRot,
                        Constants.trackWidthMeters
                ),
                Constants.trackWidthMeters,
                VecBuilder.fill(1.0, 1.0, 1.0, 1.0, 1.0),
                VecBuilder.fill(9.0, 9.0),
                0.2
        );

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(),
                                    translation.getY(),
                                    rotation,
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(),
                                    translation.getY(),
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void tankDrive(double leftPower, double rightPower){
        drive.tankDrive(leftPower, rightPower);
    }

    public Command autoBalance() {
        pid.setSetpoint(0);
        pid.setTolerance(3);

        return new RunCommand(
                () -> {
                    double power = pid.calculate(gyro.getPitch());
                    double translationVal = MathUtil.applyDeadband(-power, Constants.stickDeadband);
                    drive(new Translation2d(translationVal, 0).times(Constants.Swerve.maxSpeed),0,false, true);
                }
        );
    }

    public void setHeading(double theta) {
        Pose2d pose = getPose();

        new PPSwerveCommand(
                this,
                false,
                PathPlanner.generatePath(
                        new PathConstraints(Constants.Swerve.maxSpeed, 5),
                        List.of(new PathPoint(pose.getTranslation(), Rotation2d.fromDegrees(theta)))
                )
        ).schedule();
    }

/*
    Untested

    public Command getAutoBalancing() {
        return new RunCommand(
                () -> {
                    var v = controller.calculate(
                            getPose(),
                            (mSwerveMods[0].getState().speedMetersPerSecond + mSwerveMods[2].getState().speedMetersPerSecond)/2,
                            (mSwerveMods[1].getState().speedMetersPerSecond + mSwerveMods[3].getState().speedMetersPerSecond)/2,
                            new Pose2d(),
                            0,
                            0
                    );
                    left.setVoltage(v.left);
                    right.setVoltage(v.right);

                    for (SwerveModule mod : mSwerveMods) {
                        mod.setAngle(new SwerveModuleState(42069, Rotation2d.fromDegrees(0))); // dummy value to bypass jitter checks in setangle
                    }

                    drive.feed();
                },
                this
        ).until(() -> -Math.abs(gyro.getPitch())<=0.5);
    }
 */

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());
        drive.feed();

        SmartDashboard.putNumber("Pitch", gyro.getPitch());
        pitchEntry.setDouble(gyro.getPitch());
        pEffect.setDouble(kP * pid.getPositionError());
        dEffect.setDouble(kD * pid.getVelocityError());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}