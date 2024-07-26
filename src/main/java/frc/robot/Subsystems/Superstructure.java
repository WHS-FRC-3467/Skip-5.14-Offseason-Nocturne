// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Stage.StageSubsystem;
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
    public RobotState m_SuperState = new RobotState(IntakeSubsystem.State.OFF, StageSubsystem.State.OFF, ArmSubsystem.ArmState.STOWED, ShooterSubsystem.ShooterState.STOP);

    // Declare various "options" for our superstate
    public RobotState IDLE = 
        new RobotState(IntakeSubsystem.State.OFF, StageSubsystem.State.OFF, ArmSubsystem.ArmState.STOWED, ShooterSubsystem.ShooterState.STOP);
    public RobotState INTAKE = 
        new RobotState(IntakeSubsystem.State.FWD, StageSubsystem.State.INTAKE, ArmSubsystem.ArmState.STOWED, ShooterSubsystem.ShooterState.STOP);
    public RobotState EJECT = 
        new RobotState(IntakeSubsystem.State.REV, StageSubsystem.State.REV, ArmSubsystem.ArmState.STOWED, ShooterSubsystem.ShooterState.STOP);
    public RobotState SUBWOOFER = 
        new RobotState(IntakeSubsystem.State.OFF, StageSubsystem.State.SHOOTING, ArmSubsystem.ArmState.SUBWOOFER, ShooterSubsystem.ShooterState.SUBWOOFER);
    public RobotState PODIUM = 
        new RobotState(IntakeSubsystem.State.OFF, StageSubsystem.State.SHOOTING, ArmSubsystem.ArmState.PODIUM, ShooterSubsystem.ShooterState.SHOOT);
    public RobotState WING = 
        new RobotState(IntakeSubsystem.State.OFF, StageSubsystem.State.SHOOTING, ArmSubsystem.ArmState.WING, ShooterSubsystem.ShooterState.SHOOT);
    public RobotState AMP = 
        new RobotState(IntakeSubsystem.State.OFF, StageSubsystem.State.AMP, ArmSubsystem.ArmState.AMP, ShooterSubsystem.ShooterState.AMP);
    public RobotState CLIMB = 
        new RobotState(IntakeSubsystem.State.OFF, StageSubsystem.State.OFF, ArmSubsystem.ArmState.CLIMB, ShooterSubsystem.ShooterState.STOP);
    public RobotState HARMONY = 
        new RobotState(IntakeSubsystem.State.OFF, StageSubsystem.State.OFF, ArmSubsystem.ArmState.HARMONY, ShooterSubsystem.ShooterState.STOP);
    public RobotState AIMING = 
        new RobotState(IntakeSubsystem.State.OFF, StageSubsystem.State.OFF, ArmSubsystem.ArmState.AIMING, ShooterSubsystem.ShooterState.SHOOT); // Tentative
    public RobotState FEED = 
        new RobotState(IntakeSubsystem.State.OFF, StageSubsystem.State.SHOOTING, ArmSubsystem.ArmState.FEED, ShooterSubsystem.ShooterState.FEED);
    public RobotState PASSTHROUGH = 
        new RobotState(IntakeSubsystem.State.OFF, StageSubsystem.State.SHOOTING, ArmSubsystem.ArmState.FEED, ShooterSubsystem.ShooterState.PASSTHROUGH);

    /** Arm subsystem reference. */
    private final ArmSubsystem m_ArmSubsystem;

    /** Intake subsystem reference. */
    private final IntakeSubsystem m_IntakeSubsystem;

    /** Shooter subsystem reference. */
    private final ShooterSubsystem m_ShooterSubsystem;

    /** Stage subsystem reference. */
    private final StageSubsystem m_StageSubsystem;

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

        m_ArmSubsystem = ArmSubsystem.getInstance();
        m_IntakeSubsystem = IntakeSubsystem.getInstance();
        m_ShooterSubsystem = ShooterSubsystem.getInstance();
        m_StageSubsystem = StageSubsystem.getInstance();
        m_Drivetrain = CommandSwerveDrivetrain.getInstance();
    
        // Set Arm to Stow
        m_ArmSubsystem.setStateCommand(m_SuperState.getArmState());

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putString("Superstructure State", getM_SuperState().toString());

        // TODO: Call (and create) commands that move the subsystems to their states according to the SuperState
    }


    /**
     * @param state - use one of the defined states (INTAKE, EJECT, SUBWOOFER, etc)
     * @return a startEnd command that sets m_Superstate to the parameter for the duration of the command
    */
    public Command setStateCommand(RobotState state) {
        return startEnd(() -> setM_SuperState(state),() -> setM_SuperState(IDLE));
    }
}