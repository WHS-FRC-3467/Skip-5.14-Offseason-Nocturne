// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;
import frc.robot.Util.RobotState;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Superstructure extends SubsystemBase {

    
    @Getter
    @Setter
    RobotState m_SuperState = new RobotState(Intake.State.OFF, Stage.State.OFF, Arm.ArmState.STOWED, Shooter.ShooterState.STOP);

    // Declare various "options" for our superstate
    RobotState INTAKE = new RobotState(Intake.State.FWD, Stage.State.INTAKE, Arm.ArmState.STOWED, Shooter.ShooterState.STOP);
    RobotState EJECT = new RobotState(Intake.State.REV, Stage.State.REV, Arm.ArmState.STOWED, Shooter.ShooterState.STOP);
    RobotState SUBWOOFER = new RobotState(Intake.State.OFF, Stage.State.SHOOTING, Arm.ArmState.SUBWOOFER, Shooter.ShooterState.SUBWOOFER);
    RobotState PODIUM = new RobotState(Intake.State.OFF, Stage.State.SHOOTING, Arm.ArmState.PODIUM, Shooter.ShooterState.SHOOT);
    RobotState WING = new RobotState(Intake.State.OFF, Stage.State.SHOOTING, Arm.ArmState.WING, Shooter.ShooterState.SHOOT);
    RobotState AMP = new RobotState(Intake.State.OFF, Stage.State.AMP, Arm.ArmState.AMP, Shooter.ShooterState.AMP);
    RobotState CLIMB = new RobotState(Intake.State.OFF, Stage.State.OFF, Arm.ArmState.CLIMB, Shooter.ShooterState.STOP);
    RobotState HARMONY = new RobotState(Intake.State.OFF, Stage.State.OFF, Arm.ArmState.HARMONY, Shooter.ShooterState.STOP);
    RobotState AIMING = new RobotState(Intake.State.OFF, Stage.State.OFF, Arm.ArmState.AIMING, Shooter.ShooterState.SHOOT); // Tentative
    RobotState FEED = new RobotState(Intake.State.OFF, Stage.State.SHOOTING, Arm.ArmState.FEED, Shooter.ShooterState.FEED);
    RobotState PASSTHROUGH = new RobotState(Intake.State.OFF, Stage.State.SHOOTING, Arm.ArmState.FEED, Shooter.ShooterState.PASSTHROUGH);


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