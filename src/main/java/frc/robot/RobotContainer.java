// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SimpleSubsystem;
import frc.robot.subsystems.Stage;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
      // The robot's subsystems and commands are defined here...
    private final Intake intakeSubsystem = new Intake();
    private final Shooter m_ShooterSubsystem = new Shooter();
    private final Stage m_StageSubsystem = new Stage();
    private final Arm m_ArmSubsystem;
    private final Superstructure m_Superstructure = new Superstructure();
    private final RobotState m_RobotState;
    // Instantiate driver and operator controllers
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandXboxController m_operatorController = new CommandXboxController(1);
      GenericHID m_driveRmbl = joystick.getHID();
    GenericHID m_operatorRmbl = m_operatorController.getHID();
  public final Drivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

        // AUTO - Register Named Commands
            // Bring Arm Down, Intake Note until Note in Stage
    /* NamedCommands.registerCommand("Intake", m_ArmSubsystem.setStateCommand(Arm.ArmState.INTAKE)
            .until(m_ArmSubsystem.isArmAtState())
            .andThen(m_IntakeSubsystem.setStateCommand(Intake.State.FWD)
                    .alongWith(m_StageSubsystem.setStateCommand(Stage.State.INTAKE))
                    .until(() -> m_StageSubsystem.beambreakSupplier.getAsBoolean()))); */
        // Get Arm and Shooter Ready for Subwoofer, then Shoot
    /* NamedCommands.registerCommand("ToSubwoofer", m_ShooterSubsystem.setStateCommand(Shooter.ShooterState.SUBWOOFER)
            .alongWith(m_ArmSubsystem.setStateCommand(Arm.ArmState.SUBWOOFER))
            .alongWith(m_RobotState.setTarget(RobotState.Target.SPEAKER))
            .until(() -> (m_ShooterSubsystem.isShooterAtSpeed()
                    && m_ArmSubsystem.isArmAtState().getAsBoolean()))
                    .andThen(m_StageSubsystem.setStateCommand(Stage.State.SHOOTING))
                    .until(() -> !m_StageSubsystem.beambreakSupplier.getAsBoolean()));       */
                    
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("Tests");

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));


    
    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

            // Bindings that would be used in a match
        // Schedule `runShooterCommand` when the Xbox controller's B button is pressed,cancelling on release.
        // joystick.b().whileTrue(m_ShooterSubsystem.setStateCommand(ShooterState.SHOOT));
        // A button: stop shooter
        // joystick.a().onTrue(m_ShooterSubsystem.setStateCommand(ShooterState.STOP));
        // Manual Intake
        // joystick.x().whileTrue(intakeSubsystem.setStateCommand(Intake.State.FWD));
        // Manual shoot note (run stage)

        /* joystick.y().whileTrue(m_StageSubsystem.setStateCommand(Stage.State.SHOOTING)
            .until(() -> !m_StageSubsystem.beambreakSupplier.getAsBoolean())); */
        // Intake Note Command
        /* 
        joystick.leftTrigger(Constants.OperatorConstants.triggerThreshold).whileTrue(m_ArmSubsystem.setStateCommand(Arm.ArmState.INTAKE)
                .until(m_ArmSubsystem.isArmAtState())
                .andThen(intakeSubsystem.setStateCommand(Intake.State.FWD)
                        .alongWith(m_StageSubsystem.setStateCommand(Stage.State.INTAKE))
                        .until(() -> m_StageSubsystem.beambreakSupplier.getAsBoolean())));   */
        /* 
                        // Expel note - Manual outtake
        joystick.rightTrigger(Constants.OperatorConstants.triggerThreshold).whileTrue(intakeSubsystem.setStateCommand(Intake.State.REV)
                .alongWith(m_StageSubsystem.setStateCommand(Stage.State.REV))); */
        // Driver: Right Bumper: Arm/Shooter to FEED
        /* 
        joystick.rightBumper().whileTrue(                
            m_ShooterSubsystem.setStateCommand(Shooter.ShooterState.FEED)
                        .alongWith(m_ArmSubsystem.setStateCommand(Arm.ArmState.FEED))
                        .alongWith(m_RobotState.setTarget(RobotState.Target.FEED)));
        */
        // Operator Controls
            // Operator: DPad Left: Arm to Podium position (when pressed)
        /* m_operatorController.povLeft().whileTrue(m_ShooterSubsystem.setStateCommand(Shooter.ShooterState.SHOOT)
            .alongWith(m_ArmSubsystem.setStateCommand(Arm.ArmState.PODIUM))
            .alongWith(m_RobotState.setTarget(RobotState.Target.SPEAKER))); */

        // Operator: DPad Up: Shooter/Arm to AMP Position & Speed (when pressed)
        /*
        m_operatorController.povUp().whileTrue(
                m_ShooterSubsystem.setStateCommand(Shooter.ShooterState.AMP)
                        .alongWith(m_ArmSubsystem.setStateCommand(Arm.ArmState.AMP))
                        .alongWith(m_RobotState.setTarget(RobotState.Target.AMP)));
                      */

        // Operator: DPad Right: Arm to Harmony Position (when pressed)
        m_operatorController.povRight().whileTrue(                
                m_ArmSubsystem.setStateCommand(Arm.ArmState.HARMONY));

        // Operator: DPad Down: Arm to Subwoofer Position (when pressed)
        /*m_operatorController.povDown().whileTrue(                
            m_ShooterSubsystem.setStateCommand(Shooter.ShooterState.SUBWOOFER)
                        .alongWith(m_ArmSubsystem.setStateCommand(Arm.ArmState.SUBWOOFER)));
                           .until(() -> (m_ShooterSubsystem.isShooterAtSpeed()
                                && m_ArmSubsystem.isArmAtState().getAsBoolean()
                                 .andThen(m_StageSubsystem.setStateCommand(Stage.State.SHOOTING)))) */

    drivetrain.registerTelemetry(logger::telemeterize);

  }

  public RobotContainer() {
    m_ArmSubsystem = Arm.getInstance();
    m_RobotState = RobotState.getInstance();
    configureBindings();
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return runAuto;
  }
}
