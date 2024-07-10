// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.OperatorConstants;
//import frc.robot.Commands.Autos;
//import frc.robot.Commands.ExampleCommand;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Stage.StageSubsystem;
import frc.robot.Subsystems.LED.LEDSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Util.CommandXboxPS5Controller;
import frc.robot.Vision.Limelight;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final CommandSwerveDrivetrain m_Drivetrain = TunerConstants.DriveTrain;
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    private final StageSubsystem m_StageSubsystem = new StageSubsystem();
    private final Limelight m_LimeLight = new Limelight("ll");
    // Instantiate driver and operator controllers
    CommandXboxPS5Controller m_driverController = new CommandXboxPS5Controller(0);
    CommandXboxPS5Controller m_operatorController = new CommandXboxPS5Controller(1);
    GenericHID m_driveRmbl = m_driverController.getHID();
    GenericHID m_operatorRmbl = m_operatorController.getHID();
    // Declare the auto chooser
    private final SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
            // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
            // Another option that allows you to specify the default auto by its name
            // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Set Default Commands
        m_IntakeSubsystem.setDefaultCommand(m_IntakeSubsystem.stopIntakeCommand());
        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        /*
         * new Trigger(m_IntakeSubsystem::exampleCondition)
         * .onTrue(new intakeFwdCommand(0.7));
         */

        // Schedule `runShooterCommand` when the Xbox controller's B button is pressed,cancelling on release.
        m_driverController.b().onTrue(m_ShooterSubsystem.runShooterCommand());
        // A button: stop shooter
        m_driverController.a().onTrue(m_ShooterSubsystem.stopShooterCommand());
        // Manual Intake
        m_driverController.x().whileTrue(m_IntakeSubsystem.intakeFwdCommand(0.7));
        // Expel note - Manual outtake
        m_driverController.rightTrigger().whileTrue(new ParallelCommandGroup(m_IntakeSubsystem.intakeRevCommand(), m_StageSubsystem.reverseStageCommand()));
        // Once the button is lifted, the intake should go back to its default command
        // TODO: Make left bumper slow drivetrain
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
     
}