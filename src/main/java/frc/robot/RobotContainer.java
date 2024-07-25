// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Arm.ArmSubsystem.ArmState;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Drivetrain.Telemetry;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem.ShooterState;
import frc.robot.Subsystems.Stage.StageSubsystem;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Util.RobotState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
// import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Util.CommandXboxPS5Controller;
import frc.robot.Vision.Limelight;
import frc.robot.generated.TunerConstants;
import lombok.Getter;
import lombok.Setter;
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
    // Declare the auto chooser
    private final SendableChooser<Command> autoChooser;
    // From CTRE swerve example
    private SendableChooser<Double> speedChooser = new SendableChooser<>();
    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // Initial max is true top speed
    private final double TurtleSpeed = 0.5; // Reduction in speed from Max Speed, 0.1 = 10%, 0.5 = 50%
    private final double MaxAngularRate = Math.PI * 1.5; // .75 rotation per second max angular velocity.  Adjust for max turning rate speed.
    private final double TurtleAngularRate = Math.PI * 0.5; // .75 rotation per second max angular velocity.  Adjust for max turning rate speed.
    private double AngularRate = MaxAngularRate; // This will be updated when turtle and reset to MaxAngularRate

    // The robot's subsystems and commands are defined here...
    public final CommandSwerveDrivetrain m_Drivetrain = TunerConstants.DriveTrain;
    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    private final StageSubsystem m_StageSubsystem = new StageSubsystem();
    private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
    private final Limelight m_LimeLight = new Limelight("ll");
    // Instantiate driver and operator controllers
    CommandXboxPS5Controller m_driverController = new CommandXboxPS5Controller(OperatorConstants.kDriverControllerPort);
    CommandXboxPS5Controller m_operatorController = new CommandXboxPS5Controller(OperatorConstants.kOperatorControllerPort);
    GenericHID m_driveRmbl = m_driverController.getHID();
    GenericHID m_operatorRmbl = m_operatorController.getHID();
  
    // Field-centric driving in Open Loop, can change to closed loop after characterization 
    // For closed loop replace DriveRequestType.OpenLoopVoltage with DriveRequestType.Velocity
    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withDeadband(MaxSpeed * 0.1) // Deadband is handled on input
        .withRotationalDeadband(AngularRate * 0.1);

    SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    Telemetry logger = new Telemetry(MaxSpeed);

    Pose2d odomStart = new Pose2d(0, 0, new Rotation2d(0, 0));

    private Supplier<SwerveRequest> controlStyle;

    private Double lastSpeed = 0.65;
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Detect if controllers are missing / Stop multiple warnings
        DriverStation.silenceJoystickConnectionWarning(true);
        
        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
            // Another option that allows you to specify the default auto by its name
            // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        speedChooser.addOption("100%", 1.0);
        speedChooser.addOption("95%", 0.95);
        speedChooser.addOption("90%", 0.9);
        speedChooser.addOption("85%", 0.85);
        speedChooser.addOption("80%", 0.8);
        speedChooser.addOption("75%", 0.75);
        speedChooser.addOption("70%", 0.7);
        speedChooser.setDefaultOption("65%", 0.65);
        speedChooser.addOption("60%", 0.6);
        speedChooser.addOption("55%", 0.55);
        speedChooser.addOption("50%", 0.5);
        speedChooser.addOption("35%", 0.35);
        SmartDashboard.putData("Speed Limit", speedChooser);

        // Set Default Commands
        m_IntakeSubsystem.setDefaultCommand(m_IntakeSubsystem.setStateCommand(IntakeSubsystem.State.OFF));
        m_StageSubsystem.setDefaultCommand(m_StageSubsystem.setStateCommand(StageSubsystem.State.OFF));
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
        // Control style: Two joysticks
        controlStyle = () -> drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward -Y
        .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(-m_driverController.getRightX() * AngularRate); // Drive counterclockwise with negative X (left)
        try {
            m_Drivetrain.getDefaultCommand().cancel();
          } catch(Exception e) {}
          m_Drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
              m_Drivetrain.applyRequest(controlStyle).ignoringDisable(true));
        newSpeed();

        m_operatorController.a().whileTrue(m_Drivetrain.applyRequest(() -> brake));
        m_operatorController.b().whileTrue(m_Drivetrain
            .applyRequest(() -> point.withModuleDirection(new Rotation2d(-m_operatorController.getLeftY(), -m_operatorController.getLeftX()))));
    
        // reset the field-centric heading on start button press
        m_operatorController.start().onTrue(m_Drivetrain.runOnce(() -> m_Drivetrain.seedFieldRelative()));

        // Turtle Mode while held (Left Bumper)
        m_driverController.leftBumper().onTrue(runOnce(() -> MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * TurtleSpeed)
        .andThen(() -> AngularRate = TurtleAngularRate));
        m_driverController.leftBumper().onFalse(runOnce(() -> MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * 1) // speedChooser.getSelected()
        .andThen(() -> AngularRate = MaxAngularRate));

        if (Utils.isSimulation()) {
            m_Drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
        }
            m_Drivetrain.registerTelemetry(logger::telemeterize);

        Trigger speedPick = new Trigger(() -> lastSpeed != speedChooser.getSelected());
        speedPick.onTrue(runOnce(() -> newSpeed()));

        // Testing uses operator controller
        m_operatorController.x().and(m_operatorController.pov(0)).whileTrue(m_Drivetrain.runDriveQuasiTest(Direction.kForward));
        m_operatorController.x().and(m_operatorController.pov(180)).whileTrue(m_Drivetrain.runDriveQuasiTest(Direction.kReverse));

        m_operatorController.y().and(m_operatorController.pov(0)).whileTrue(m_Drivetrain.runDriveDynamTest(Direction.kForward));
        m_operatorController.y().and(m_operatorController.pov(180)).whileTrue(m_Drivetrain.runDriveDynamTest(Direction.kReverse));

        m_operatorController.a().and(m_operatorController.pov(0)).whileTrue(m_Drivetrain.runSteerQuasiTest(Direction.kForward));
        m_operatorController.a().and(m_operatorController.pov(180)).whileTrue(m_Drivetrain.runSteerQuasiTest(Direction.kReverse));

        m_operatorController.b().and(m_operatorController.pov(0)).whileTrue(m_Drivetrain.runSteerDynamTest(Direction.kForward));
        m_operatorController.b().and(m_operatorController.pov(180)).whileTrue(m_Drivetrain.runSteerDynamTest(Direction.kReverse));

        // Drivetrain needs to be placed against a sturdy wall and test stopped immediately upon wheel slip
        m_operatorController.back().and(m_operatorController.pov(0)).whileTrue(m_Drivetrain.runDriveSlipTest());

        // Bindings that would be used in a match
        // Schedule `runShooterCommand` when the Xbox controller's B button is pressed,cancelling on release.
        m_driverController.b().onTrue(m_ShooterSubsystem.setStateCommand(ShooterState.SHOOT));
        // A button: stop shooter
        m_driverController.a().onTrue(m_ShooterSubsystem.setStateCommand(ShooterState.STOP));
        // Manual Intake
        m_driverController.x().onTrue(m_IntakeSubsystem.setStateCommand(IntakeSubsystem.State.FWD));
        // Intake Note Command
        m_driverController.leftTrigger().whileTrue(m_ArmSubsystem.setStateCommand(ArmState.STOWED)
            .until(m_ArmSubsystem.isArmAtState())
            .andThen(new ParallelCommandGroup(m_IntakeSubsystem.setStateCommand(IntakeSubsystem.State.FWD), m_StageSubsystem.runStageUntilNoteCommand())));
        // Expel note - Manual outtake
        m_driverController.rightTrigger().whileTrue(new ParallelCommandGroup(m_IntakeSubsystem.setStateCommand(IntakeSubsystem.State.REV), m_StageSubsystem.setStateCommand(StageSubsystem.State.REV)));
        // Once the button is lifted, the intake should go back to its default command
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    
    private void newSpeed() {
        lastSpeed = speedChooser.getSelected();
        MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * lastSpeed;
      }
    
}