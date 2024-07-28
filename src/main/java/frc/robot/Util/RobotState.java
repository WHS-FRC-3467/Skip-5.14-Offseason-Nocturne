// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.util.function.DoubleSupplier;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Stage;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

/**
 * This comment is copied from main
 * A Class to hold setpoints for both the Shooter and the Arm, since they work
 * in tandem.
 * @param intake  The Intake state
 * @param stage   The Stage state
 * @param arm     The Arm setpoint
 * @param shooter The Shooter setpoint
 */

    public class RobotState {
        // TODO: Integrate Shooter and Arm state/setpoint into combined setpoints
        Intake.State INTAKE;
        Stage.State STAGE;
        Arm.ArmState ARM;
        Shooter.ShooterState SHOOTER;

        public RobotState(Intake.State intakestate, Stage.State stagestate, Arm.ArmState armstate, Shooter.ShooterState shooterstate){
            this.INTAKE = intakestate;
            this.STAGE = stagestate;
            this.ARM = armstate;
            this.SHOOTER = shooterstate;

        }
    }