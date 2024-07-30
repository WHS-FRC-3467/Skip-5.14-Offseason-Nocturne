// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Subsystems.Arm.ArmState;
import frc.robot.Subsystems.Shooter.ShooterState;
import frc.robot.Subsystems.Stage.State;
import frc.robot.RobotContainer;
import frc.robot.Util.RobotState;
import frc.robot.Vision.Limelight;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Superstructure extends SubsystemBase {

    /** Superstucture singleton. */
    private static Superstructure instance = null;

    @Getter
    @Setter
    public RobotState m_SuperState = new RobotState(Intake.State.OFF, Stage.State.OFF, Arm.ArmState.STOWED, Shooter.ShooterState.STOP);

    // Declare various "options" for our superstate
    public RobotState IDLE = 
        new RobotState(Intake.State.OFF, Stage.State.OFF, Arm.ArmState.STOWED, Shooter.ShooterState.STOP);
    public RobotState INTAKE = 
        new RobotState(Intake.State.FWD, Stage.State.INTAKE, Arm.ArmState.STOWED, Shooter.ShooterState.STOP);
    public RobotState EJECT = 
        new RobotState(Intake.State.REV, Stage.State.REV, Arm.ArmState.STOWED, Shooter.ShooterState.STOP);
    public RobotState SUBWOOFER = 
        new RobotState(Intake.State.OFF, Stage.State.SHOOTING, Arm.ArmState.SUBWOOFER, Shooter.ShooterState.SUBWOOFER);
    public RobotState PODIUM = 
        new RobotState(Intake.State.OFF, Stage.State.SHOOTING, Arm.ArmState.PODIUM, Shooter.ShooterState.SHOOT);
    public RobotState WING = 
        new RobotState(Intake.State.OFF, Stage.State.SHOOTING, Arm.ArmState.WING, Shooter.ShooterState.SHOOT);
    public RobotState AMP = 
        new RobotState(Intake.State.OFF, Stage.State.AMP, Arm.ArmState.AMP, Shooter.ShooterState.AMP);
    public RobotState CLIMB = 
        new RobotState(Intake.State.OFF, Stage.State.OFF, Arm.ArmState.CLIMB, Shooter.ShooterState.STOP);
    public RobotState HARMONY = 
        new RobotState(Intake.State.OFF, Stage.State.OFF, Arm.ArmState.HARMONY, Shooter.ShooterState.STOP);
    public RobotState AIMING = 
        new RobotState(Intake.State.OFF, Stage.State.OFF, Arm.ArmState.AIMING, Shooter.ShooterState.SHOOT); // Tentative
    public RobotState FEED = 
        new RobotState(Intake.State.OFF, Stage.State.SHOOTING, Arm.ArmState.FEED, Shooter.ShooterState.FEED);
    public RobotState PASSTHROUGH = 
        new RobotState(Intake.State.OFF, Stage.State.SHOOTING, Arm.ArmState.FEED, Shooter.ShooterState.PASSTHROUGH);

    /** Arm subsystem reference. */
    private final Arm m_ArmSubsystem;

    /** Intake subsystem reference. */
    private final Intake m_IntakeSubsystem;

    /** Shooter subsystem reference. */
    private final Shooter m_ShooterSubsystem;

    /** Stage subsystem reference. */
    private final Stage m_StageSubsystem;

        /** Drivetrain subsystem reference. */
    private final CommandSwerveDrivetrain m_Drivetrain;
    
    /**
    * Returns the superstructure instance.
    *
    * @return the superstructure instance.
    */
    public static Superstructure getInstance() {
        if (instance == null) {
        instance = new Superstructure();
        }

        return instance;
    }

    /** Creates a new SuperstructureSubsystem. */
    public Superstructure() {

        m_ArmSubsystem = Arm.getInstance();
        m_IntakeSubsystem = Intake.getInstance();
        m_ShooterSubsystem = Shooter.getInstance();
        m_StageSubsystem = Stage.getInstance();
        m_Drivetrain = CommandSwerveDrivetrain.getInstance();
    
        // Set Arm to Stow
        m_ArmSubsystem.setStateCommand(m_SuperState.getArmState());

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putString("Superstructure State", getM_SuperState().toString());

        // TODO: Call (and create) commands that move the subsystems to their states according to the SuperState
        // Either do it here or in setStateCommand
    }


    // Command factories. Public as of right now for direct use with Robot Container.

    public Command manualIntakeCommand() {
        return m_IntakeSubsystem.setStateCommand(Intake.State.FWD);
    }

    /**
     * 1. Move Arm Down to STOW
     * 2. Then, run the Intake and Stage until the Stage Detects a Note
     * @return a command
    */
    public Command intakeCommand() {
        return m_ArmSubsystem.setStateCommand(ArmState.STOWED)
            .until(m_ArmSubsystem.isArmAtState())
            .andThen(new ParallelCommandGroup(m_IntakeSubsystem.setStateCommand(Intake.State.FWD), m_StageSubsystem.runStageUntilNoteCommand()));
    }

    public Command ejectCommand() {
        return new ParallelCommandGroup(m_IntakeSubsystem.setStateCommand(Intake.State.REV), m_StageSubsystem.setStateCommand(Stage.State.REV));
    }

    public Command startShooterCommand() {
        return m_ShooterSubsystem.setStateCommand(ShooterState.SHOOT);
    }

    public Command stopShooterCommand() {
        return m_ShooterSubsystem.setStateCommand(ShooterState.STOP);
    }

    /**
     * Runs stage wheels until beambreak does not detect note in stage
     * @return a command that does not end until note leaves stage
    */
    public Command shootCommand() {
        return m_StageSubsystem.runStageUntilNoNoteCommand();
    }

        // Operator: DPad Left: Arm to Podium position (when pressed)
    public Command armToPodiumCommand() {
        if (m_StageSubsystem.isNoteInStage()) {
            return new ParallelCommandGroup(
                m_ShooterSubsystem.setStateCommand(ShooterState.SHOOT), 
                m_ArmSubsystem.setStateCommand(ArmState.PODIUM));
        } else {
            return null;
        }
    }
    // Operator: DPad Up: Shooter/Arm to AMP Position & Speed (when pressed)
    public Command armToAmpCommand() {
        if (m_StageSubsystem.isNoteInStage()) {
            return new ParallelCommandGroup(
                m_ShooterSubsystem.setStateCommand(ShooterState.AMP), 
                m_ArmSubsystem.setStateCommand(ArmState.AMP))
                .until(()-> (m_ShooterSubsystem.isShooterAtSpeed() && m_ArmSubsystem.isArmAtState().getAsBoolean()));
        } else {
            return null;
        }
    }

    // Operator: DPad Right: Arm to Harmony Position (when pressed)
    public Command armToHarmonyCommand() {
        return new ParallelCommandGroup(
            m_ShooterSubsystem.setStateCommand(ShooterState.STOP), 
            m_ArmSubsystem.setStateCommand(ArmState.HARMONY)).until(m_ArmSubsystem.isArmAtState())
            .until(()-> (m_ShooterSubsystem.isShooterAtSpeed() && m_ArmSubsystem.isArmAtState().getAsBoolean()));
    }

    // Operator: DPad Down: Arm to Subwoofer Position (when pressed)
    public Command armToSubCommand() {
        return new ParallelCommandGroup(
            m_ShooterSubsystem.setStateCommand(ShooterState.SUBWOOFER), 
            m_ArmSubsystem.setStateCommand(ArmState.SUBWOOFER))
            .until(()-> (m_ShooterSubsystem.isShooterAtSpeed() && m_ArmSubsystem.isArmAtState().getAsBoolean()));
    }
    
    /**
     * @param state - use one of the defined states (INTAKE, EJECT, SUBWOOFER, etc)
     * @return a startEnd command that sets m_Superstate to the parameter for the duration of the command
    */
    public Command setStateCommand(RobotState state) {
        return startEnd(() -> setM_SuperState(state), () -> setM_SuperState(IDLE));
    }
}