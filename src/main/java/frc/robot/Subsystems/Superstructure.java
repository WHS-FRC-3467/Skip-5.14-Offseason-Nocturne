// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Subsystems.Arm.ArmSubsystem;
import frc.robot.Subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Stage.StageSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Util.RobotState;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Superstructure extends SubsystemBase {

    
    @Getter
    @Setter
    RobotState m_SuperState = new RobotState(IntakeSubsystem.State.OFF, StageSubsystem.State.OFF, ArmSubsystem.ArmState.STOWED, ShooterSubsystem.ShooterState.STOP);

    // Declare various "options" for our superstate
    RobotState INTAKE = new RobotState(IntakeSubsystem.State.FWD, StageSubsystem.State.INTAKE, ArmSubsystem.ArmState.STOWED, ShooterSubsystem.ShooterState.STOP);
    RobotState EJECT = new RobotState(IntakeSubsystem.State.REV, StageSubsystem.State.REV, ArmSubsystem.ArmState.STOWED, ShooterSubsystem.ShooterState.STOP);
    RobotState SUBWOOFER = new RobotState(IntakeSubsystem.State.OFF, StageSubsystem.State.SHOOTING, ArmSubsystem.ArmState.SUBWOOFER, ShooterSubsystem.ShooterState.SUBWOOFER);
    RobotState PODIUM = new RobotState(IntakeSubsystem.State.OFF, StageSubsystem.State.SHOOTING, ArmSubsystem.ArmState.PODIUM, ShooterSubsystem.ShooterState.SHOOT);
    RobotState WING = new RobotState(IntakeSubsystem.State.OFF, StageSubsystem.State.SHOOTING, ArmSubsystem.ArmState.WING, ShooterSubsystem.ShooterState.SHOOT);
    RobotState AMP = new RobotState(IntakeSubsystem.State.OFF, StageSubsystem.State.AMP, ArmSubsystem.ArmState.AMP, ShooterSubsystem.ShooterState.AMP);
    RobotState CLIMB = new RobotState(IntakeSubsystem.State.OFF, StageSubsystem.State.OFF, ArmSubsystem.ArmState.CLIMB, ShooterSubsystem.ShooterState.STOP);
    RobotState HARMONY = new RobotState(IntakeSubsystem.State.OFF, StageSubsystem.State.OFF, ArmSubsystem.ArmState.HARMONY, ShooterSubsystem.ShooterState.STOP);
    RobotState AIMING = new RobotState(IntakeSubsystem.State.OFF, StageSubsystem.State.OFF, ArmSubsystem.ArmState.AIMING, ShooterSubsystem.ShooterState.SHOOT); // Tentative
    RobotState FEED = new RobotState(IntakeSubsystem.State.OFF, StageSubsystem.State.SHOOTING, ArmSubsystem.ArmState.FEED, ShooterSubsystem.ShooterState.FEED);
    RobotState PASSTHROUGH = new RobotState(IntakeSubsystem.State.OFF, StageSubsystem.State.SHOOTING, ArmSubsystem.ArmState.FEED, ShooterSubsystem.ShooterState.PASSTHROUGH);


    /** Creates a new SuperstructureSubsystem. */
    public Superstructure() {


    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run


    }


    // Set m_SuperState to be one of the defined states (intake, eject, sub, etc)
    public Command setStateCommand(RobotState state) {
        return startEnd(() -> setM_SuperState(state),() -> setM_SuperState(m_SuperState));
    }
}