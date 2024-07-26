package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem {

    @RequiredArgsConstructor
    @Getter
    public enum State {
        TELEOP,
        HEADING,
        SHOOTONTHEMOVE;

    }

    @Setter
    private State state = State.TELEOP;

    public State getDrivetrainState() {
        return state;
    }

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    private Optional<Rotation2d> headingAngle = Optional.empty();
    private double controllerX = 0.0;
    private double controllerY = 0.0;
    private double controllerOmega = 0.0;

    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) 
            .withVelocityX(controllerX * MaxSpeed)
            .withVelocityY(controllerY * MaxSpeed)
            .withRotationalRate(controllerOmega * MaxAngularRate);

    private SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) 
            .withDriveRequestType(DriveRequestType.Velocity) 
            .withVelocityX(controllerX * MaxSpeed)
            .withVelocityY(controllerY * MaxSpeed)
            .withTargetDirection(new Rotation2d());

    //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public Drivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        setHeadingPID();
        setSwerveDriveCustomCurrentLimits();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                                                                             // robot
                new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, // Assume the path needs to be
                                                                                         // flipped for Red vs Blue,
                                                                                         // this is normally the case
                this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void simulationPeriodic() {
        /* Assume 20ms update rate, get battery voltage from WPILib */
        updateSimState(0.02, RobotController.getBatteryVoltage());
    }

    @Override
    public void periodic() {

        switch (state) {
            case TELEOP -> {
                this.setControl(fieldCentric);
            }
            case HEADING -> {
                this.setControl(fieldCentricFacingAngle);
            }
            case SHOOTONTHEMOVE -> {
                this.setControl(fieldCentricFacingAngle);
            }
            default -> {}
        }

    }

    private void setHeadingPID() {
        fieldCentricFacingAngle.HeadingController.setPID(25, 10, 2);
        fieldCentricFacingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        fieldCentricFacingAngle.HeadingController.setTolerance(Units.degreesToRadians(.5));
    }

    // Gets current rotation from estimated pose
/*     public Rotation2d getRotation() {
        return getState().Pose.getRotation();
    } */

    public void setControllerInput(double controllerX, double controllerY, double controllerOmega) {
        this.controllerX = controllerX;
        this.controllerY = controllerY;
        this.controllerOmega = controllerOmega;
    }

    public void setHeadingAngle(Rotation2d angle) {
        this.headingAngle = Optional.ofNullable(angle);
    }

    public void clearHeadingAngle() {
        this.headingAngle = null;
    }

    public boolean atHeadingAngle() {
        return fieldCentricFacingAngle.HeadingController.atSetpoint();
    }

    public Command setStateCommand(State state) {
        return startEnd(() -> setState(state),() -> setState(State.TELEOP));
      }


      public void setSwerveDriveCustomCurrentLimits() {
          // Create a current configuration to use for the drive motor of each swerve
          // module.
          var customCurrentLimitConfigs = new CurrentLimitsConfigs();

          // Iterate through each module.
          for (var module : Modules) {
              // Get the Configurator for the current drive motor.
              var currentConfigurator = module.getDriveMotor().getConfigurator();

              // Refresh the current configuration, since the stator current limit has already
              // been set.
              currentConfigurator.refresh(customCurrentLimitConfigs);

              // Set all of the parameters related to the supply current. The values should
              // come from Constants.
              customCurrentLimitConfigs.SupplyCurrentLimit = 30;
              customCurrentLimitConfigs.SupplyCurrentThreshold = 90;
              customCurrentLimitConfigs.SupplyTimeThreshold = .01;
              customCurrentLimitConfigs.SupplyCurrentLimitEnable = true;

              customCurrentLimitConfigs.StatorCurrentLimit = 80;
              customCurrentLimitConfigs.StatorCurrentLimitEnable = true;

              // Apply the new current limit configuration.
              currentConfigurator.apply(customCurrentLimitConfigs);
          }
      }
}
